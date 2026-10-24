"""LiDAR depth image: python -m go2_lidar.depth

Subscribes to the bridge's LiDAR stream and renders a front-view depth
image, as if seen from the Go2's front camera: each pixel is the forward
distance (metres) of the nearest LiDAR point in that direction.

The L1 LiDAR is mounted almost upside down in the Go2's chin (URDF
radar_joint: pitch 2.8782 rad), so points are first transformed from the
sensor frame into the robot body frame (x forward, y left, z up). The L1 is
sparse per scan, so the last few clouds are accumulated and small gaps are
filled with the nearest neighbouring point.

Shows the colourised image in a window and, if GO2_DEPTH_SAVE is set, writes
it there (plus a 16-bit depth PNG in millimetres next to it) once per second.
"""

from __future__ import annotations

import collections
import json
import logging
import math
import os
import time

import cv2
import numpy as np
import zmq

BRIDGE_HOST = os.getenv("GO2_BRIDGE_HOST", "localhost")
LIDAR_PORT = int(os.getenv("GO2_ZMQ_LIDAR_PORT", "5558"))
# sensor → body transform "x y z roll pitch yaw" (m, rad), default from Unitree's go2_description URDF
LIDAR_MOUNT = [float(v) for v in os.getenv("GO2_LIDAR_MOUNT", "0.28945 0 -0.046825 0 2.8782 0").split()]
# virtual camera position in the body frame, default = front camera in the URDF
DEPTH_CAMERA_POS = np.array([float(v) for v in os.getenv("GO2_DEPTH_CAMERA_POS", "0.32715 0 0.04297").split()])
# kept small on purpose: the L1 only gives ~1–2k points per scan, most of them outside the front view
DEPTH_WIDTH = int(os.getenv("GO2_DEPTH_WIDTH", "320"))
DEPTH_HEIGHT = int(os.getenv("GO2_DEPTH_HEIGHT", "180"))
DEPTH_HFOV = math.radians(float(os.getenv("GO2_DEPTH_HFOV", "90")))  # horizontal field of view
DEPTH_MAX_RANGE = float(os.getenv("GO2_DEPTH_MAX_RANGE", "10"))  # metres, for colouring
DEPTH_ACCUMULATE = int(os.getenv("GO2_DEPTH_ACCUMULATE", "20"))  # clouds to merge (~1.3 s at 15 Hz)
DEPTH_FILL = int(os.getenv("GO2_DEPTH_FILL", "9"))  # max gap-fill window size, 0 = off
DEPTH_SHOW = os.getenv("GO2_DEPTH_SHOW", "1") == "1"
DEPTH_SAVE = os.getenv("GO2_DEPTH_SAVE", "")  # e.g. depth.png

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("go2_lidar.depth")

# sensor_msgs/PointField datatype → NumPy type
_POINTFIELD_TYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def cloud_to_xyz(meta: dict, data: bytes) -> np.ndarray:
    """Decode a PointCloud2 buffer into an (N, 3) float32 array of finite x, y, z."""
    fields = {f["name"]: f for f in meta["fields"]}
    order = ">" if meta["is_bigendian"] else "<"
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [order + _POINTFIELD_TYPES[fields[n]["datatype"]] for n in "xyz"],
        "offsets": [fields[n]["offset"] for n in "xyz"],
        "itemsize": meta["point_step"],
    })
    cloud = np.frombuffer(data, dtype=dtype, count=meta["width"] * meta["height"])
    xyz = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def mount_transform(x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
    """URDF-style origin → (R, t) with p_body = R @ p_sensor + t."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return (rz @ ry @ rx).astype(np.float32), np.array([x, y, z], dtype=np.float32)


def render_front_depth(xyz_body: np.ndarray, width: int, height: int, hfov: float,
                       cam_pos: np.ndarray, fill: int = 0) -> np.ndarray:
    """Pinhole projection looking along +x from cam_pos (metres, 0 = no return).

    Pixel value is the forward distance (x) of the nearest point; image left =
    robot's left (+y), image up = +z.
    """
    p = xyz_body - cam_pos
    p = p[p[:, 0] > 0.05]  # in front of the camera
    f = (width / 2) / math.tan(hfov / 2)
    u = (width / 2 - f * p[:, 1] / p[:, 0]).astype(int)
    v = (height / 2 - f * p[:, 2] / p[:, 0]).astype(int)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, d = u[inside], v[inside], p[inside, 0]

    depth = np.full((height, width), np.inf, dtype=np.float32)
    order = np.argsort(-d)  # far first, so nearer points overwrite them
    depth[v[order], u[order]] = d[order]

    # fill only empty pixels (inf) with the nearest depth around them, growing the
    # window step by step; measured pixels are left untouched
    for k in range(3, fill + 1, 2):
        empty = np.isinf(depth)
        if not empty.any():
            break
        grown = cv2.erode(depth, np.ones((k, k), np.uint8))  # erosion = min filter
        depth[empty] = grown[empty]

    depth[np.isinf(depth)] = 0.0
    return depth


def colorize(depth: np.ndarray, max_range: float) -> np.ndarray:
    """Near = warm, far = cold, no return = black."""
    norm = 1.0 - np.clip(depth / max_range, 0.0, 1.0)
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[depth <= 0] = 0
    return img


def main() -> None:
    rot, trans = mount_transform(*LIDAR_MOUNT)

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVTIMEO, 1000)  # short, so Ctrl+C is handled promptly
    sub.connect(f"tcp://{BRIDGE_HOST}:{LIDAR_PORT}")
    sub.subscribe(b"lidar")
    log.info("Subscribed to tcp://%s:%d", BRIDGE_HOST, LIDAR_PORT)

    recent: collections.deque[np.ndarray] = collections.deque(maxlen=max(1, DEPTH_ACCUMULATE))
    first = True
    t_rx = t_saved = t_stats = time.time()

    try:
        while True:
            try:
                _topic, meta_raw, data = sub.recv_multipart()
            except zmq.Again:
                if time.time() - t_rx >= 5.0:
                    log.warning("No LiDAR data for 5 s — is the bridge running with GO2_LIDAR=1?")
                    t_rx = time.time()
                continue
            t_rx = time.time()

            meta = json.loads(meta_raw)
            if first:
                first = False
                log.info("First cloud in frame '%s'", meta["frame_id"])
                if any(k in meta["frame_id"].lower() for k in ("odom", "world", "map")):
                    log.warning("Cloud is in a world frame, not the sensor frame — the mount transform "
                                "won't make it robot-relative and the view will drift as the robot moves")

            recent.append(cloud_to_xyz(meta, data) @ rot.T + trans)
            body = np.concatenate(recent)
            depth = render_front_depth(body, DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_HFOV,
                                       DEPTH_CAMERA_POS, DEPTH_FILL)
            img = colorize(depth, DEPTH_MAX_RANGE)

            if time.time() - t_stats >= 5.0:
                lo, hi = body.min(axis=0), body.max(axis=0)
                log.info("%d points (body frame x %.1f…%.1f, y %.1f…%.1f, z %.1f…%.1f m), %.0f%% of image filled",
                         len(body), lo[0], hi[0], lo[1], hi[1], lo[2], hi[2], 100 * (depth > 0).mean())
                t_stats = time.time()

            if DEPTH_SAVE and time.time() - t_saved >= 1.0:
                cv2.imwrite(DEPTH_SAVE, img)
                root, _ext = os.path.splitext(DEPTH_SAVE)
                cv2.imwrite(f"{root}_mm.png", (np.clip(depth, 0, 65.535) * 1000).astype(np.uint16))
                t_saved = time.time()

            if DEPTH_SHOW:
                cv2.imshow("go2 lidar depth (front)",
                           cv2.resize(img, (640, 640 * DEPTH_HEIGHT // DEPTH_WIDTH), interpolation=cv2.INTER_NEAREST))
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        ctx.term()
        if DEPTH_SHOW:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
