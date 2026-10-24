"""LiDAR depth image: python -m go2_lidar.depth

Subscribes to the bridge's LiDAR stream and renders a 360° range image
(equirectangular: azimuth across, elevation down, pixel = distance from the
sensor). The Go2's L1 LiDAR is sparse per scan, so the last few clouds are
accumulated and small gaps are filled with the nearest neighbouring point.

Shows the colourised image in a window and, if GO2_DEPTH_SAVE is set, writes
it there (plus a 16-bit depth PNG in millimetres next to it) once per second.
"""

from __future__ import annotations

import collections
import json
import logging
import os
import time

import cv2
import numpy as np
import zmq

BRIDGE_HOST = os.getenv("GO2_BRIDGE_HOST", "localhost")
LIDAR_PORT = int(os.getenv("GO2_ZMQ_LIDAR_PORT", "5558"))
DEPTH_WIDTH = int(os.getenv("GO2_DEPTH_WIDTH", "720"))  # 0.5° per pixel
DEPTH_MAX_RANGE = float(os.getenv("GO2_DEPTH_MAX_RANGE", "10"))  # metres, for colouring
DEPTH_ACCUMULATE = int(os.getenv("GO2_DEPTH_ACCUMULATE", "10"))  # clouds to merge
DEPTH_FILL = int(os.getenv("GO2_DEPTH_FILL", "5"))  # gap-fill kernel size, 0 = off
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


def render_range_image(xyz: np.ndarray, width: int, fill: int = 0) -> np.ndarray:
    """Project points into an equirectangular range image (metres, 0 = no return).

    Columns span azimuth +180°…-180° (left to right, x forward is the centre),
    rows span elevation +90°…-90° (top to bottom). The nearest point wins.
    """
    height = width // 2
    r = np.linalg.norm(xyz, axis=1)
    valid = r > 1e-3
    xyz, r = xyz[valid], r[valid]

    azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
    elevation = np.arcsin(np.clip(xyz[:, 2] / r, -1.0, 1.0))
    u = ((np.pi - azimuth) / (2 * np.pi) * width).astype(int) % width
    v = np.clip(((np.pi / 2 - elevation) / np.pi * height).astype(int), 0, height - 1)

    depth = np.full((height, width), np.inf, dtype=np.float32)
    order = np.argsort(-r)  # far first, so nearer points overwrite them
    depth[v[order], u[order]] = r[order]

    if fill > 1:
        # erosion = min filter: empty pixels (inf) take the nearest neighbouring range
        depth = cv2.erode(depth, np.ones((fill, fill), np.uint8))

    depth[np.isinf(depth)] = 0.0
    return depth


def colorize(depth: np.ndarray, max_range: float) -> np.ndarray:
    """Near = warm, far = cold, no return = black."""
    norm = 1.0 - np.clip(depth / max_range, 0.0, 1.0)
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[depth <= 0] = 0
    return img


def main() -> None:
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVTIMEO, 1000)  # short, so Ctrl+C is handled promptly
    sub.connect(f"tcp://{BRIDGE_HOST}:{LIDAR_PORT}")
    sub.subscribe(b"lidar")
    log.info("Subscribed to tcp://%s:%d", BRIDGE_HOST, LIDAR_PORT)

    recent: collections.deque[np.ndarray] = collections.deque(maxlen=max(1, DEPTH_ACCUMULATE))
    first = True
    t_rx = t_saved = time.time()

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
                log.info("First cloud in frame '%s' — ranges are measured from that frame's origin",
                         meta["frame_id"])

            recent.append(cloud_to_xyz(meta, data))
            depth = render_range_image(np.concatenate(recent), DEPTH_WIDTH, DEPTH_FILL)
            img = colorize(depth, DEPTH_MAX_RANGE)

            if DEPTH_SAVE and time.time() - t_saved >= 1.0:
                cv2.imwrite(DEPTH_SAVE, img)
                root, _ext = os.path.splitext(DEPTH_SAVE)
                cv2.imwrite(f"{root}_mm.png", (np.clip(depth, 0, 65.535) * 1000).astype(np.uint16))
                t_saved = time.time()

            if DEPTH_SHOW:
                cv2.imshow("go2 lidar depth", img)
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
