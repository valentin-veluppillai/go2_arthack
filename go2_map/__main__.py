"""Live 3D map: python -m go2_map

Subscribes to the bridge's SLAM stream (port 5559), merges every world-frame
scan from the Go2's on-board SLAM (uslam_server) into one voxel map and shows
it live in a Rerun 3D viewer, together with the robot pose, its path and the
current scan. The map keeps everything seen so far, so the whole area around
the robot stays visible.

uslam only publishes while it is mapping (or localized on a saved map), so by
default this sends "mapping/start" through the bridge at startup.
"""

from __future__ import annotations

import json
import logging
import os
import time

import numpy as np
import rerun as rr
import zmq

BRIDGE_HOST = os.getenv("GO2_BRIDGE_HOST", "localhost")
CMD_PORT = int(os.getenv("GO2_ZMQ_CMD_PORT", "5555"))
SLAM_PORT = int(os.getenv("GO2_ZMQ_SLAM_PORT", "5559"))
MAP_VOXEL = float(os.getenv("GO2_MAP_VOXEL", "0.05"))  # metres per map point
# hide points more than this far above the robot's start height, so indoor ceilings
# don't cover the map (0 = show everything)
MAP_CEILING = float(os.getenv("GO2_MAP_CEILING", "1.5"))
MAP_START = os.getenv("GO2_MAP_START", "1") == "1"  # send mapping/start at startup
MAP_WEB = os.getenv("GO2_MAP_WEB", "0") == "1"  # browser viewer instead of the native one
MAP_SAVE = os.getenv("GO2_MAP_SAVE", "")  # write a .rrd recording instead of showing it
MAP_LOG_HZ = 1.0  # full-map refresh rate in the viewer

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("go2_map")

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


class VoxelMap:
    """Keeps one point per voxel; the first point seen in a voxel stays."""

    def __init__(self, voxel: float) -> None:
        self.voxel = voxel
        self.keys = np.empty(0, dtype=np.int64)
        self.points = np.empty((0, 3), dtype=np.float32)

    def _keys(self, pts: np.ndarray) -> np.ndarray:
        idx = np.floor(pts / self.voxel).astype(np.int64) + (1 << 20)  # ±52 km at 5 cm
        return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]

    def add(self, pts: np.ndarray) -> None:
        keys = np.concatenate([self.keys, self._keys(pts)])
        points = np.concatenate([self.points, pts])
        self.keys, first = np.unique(keys, return_index=True)  # existing voxels come first
        self.points = points[first]


def below_ceiling(points: np.ndarray, z_ref: float, ceiling: float) -> np.ndarray:
    return points if ceiling <= 0 else points[points[:, 2] < z_ref + ceiling]


def height_colors(z: np.ndarray) -> np.ndarray:
    """Blue (low) → green → yellow → red (high), robust to outliers."""
    lo, hi = np.percentile(z, [2, 98]) if len(z) else (0.0, 1.0)
    t = np.clip((z - lo) / max(hi - lo, 1e-3), 0, 1)
    anchors = np.array([[40, 70, 220], [40, 190, 220], [80, 200, 80], [240, 220, 60], [230, 60, 40]])
    pos = np.linspace(0, 1, len(anchors))
    return np.stack([np.interp(t, pos, anchors[:, c]) for c in range(3)], axis=1).astype(np.uint8)


def send_slam_command(ctx: zmq.Context, command: str) -> None:
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, 3000)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{BRIDGE_HOST}:{CMD_PORT}")
    try:
        sock.send_json({"cmd": "slam", "params": {"command": command}})
        log.info("Bridge: %s", sock.recv_json().get("msg"))
    except zmq.Again:
        log.warning("Bridge did not answer — is it running?")
    finally:
        sock.close()


def init_viewer() -> None:
    rr.init("go2_map")
    if MAP_SAVE:
        rr.save(MAP_SAVE)
        log.info("Recording to %s (open with: rerun %s)", MAP_SAVE, MAP_SAVE)
    elif MAP_WEB:
        rr.serve_web_viewer(connect_to=rr.serve_grpc())
    else:
        rr.spawn()
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    # Go2 body ≈ 0.70 × 0.31 × 0.40 m, with a heading arrow
    rr.log("world/robot/body", rr.Boxes3D(half_sizes=[0.35, 0.155, 0.2], colors=[255, 140, 0]), static=True)
    rr.log("world/robot/heading", rr.Arrows3D(vectors=[[0.6, 0, 0]], colors=[255, 140, 0]), static=True)


def main() -> None:
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVTIMEO, 1000)  # short, so Ctrl+C is handled promptly
    sub.connect(f"tcp://{BRIDGE_HOST}:{SLAM_PORT}")
    for topic in (b"slam_cloud", b"slam_odom", b"slam_log"):
        sub.subscribe(topic)
    log.info("Subscribed to tcp://%s:%d", BRIDGE_HOST, SLAM_PORT)

    init_viewer()
    if MAP_START:
        time.sleep(0.3)  # let the SUB socket connect so the ack isn't missed
        send_slam_command(ctx, "mapping/start")

    voxels = VoxelMap(MAP_VOXEL)
    path: list[list[float]] = []
    z_ref: float | None = None  # robot height at the first pose, reference for the ceiling cut
    t_start = t_logged = t_rx = time.time()
    clouds = 0
    dirty = False

    try:
        while True:
            try:
                topic, *frames = sub.recv_multipart()
            except zmq.Again:
                topic = None

            if topic == b"slam_cloud":
                t_rx = time.time()
                scan = cloud_to_xyz(json.loads(frames[0]), frames[1])
                voxels.add(scan)
                rr.log("world/scan", rr.Points3D(below_ceiling(scan, z_ref or 0.0, MAP_CEILING),
                                                 colors=[255, 255, 255], radii=0.02))
                clouds += 1
                dirty = True
            elif topic == b"slam_odom":
                odom = json.loads(frames[0])
                if z_ref is None:
                    z_ref = odom["position"][2]
                rr.log("world/robot", rr.Transform3D(translation=odom["position"],
                                                     rotation=rr.Quaternion(xyzw=odom["orientation"])))
                if not path or np.linalg.norm(np.subtract(odom["position"], path[-1])) > 0.05:
                    path.append(odom["position"])
                    dirty = True
            elif topic == b"slam_log":
                log.info("uslam: %s", frames[0].decode(errors="replace"))

            now = time.time()
            if dirty and now - t_logged >= 1.0 / MAP_LOG_HZ:
                shown = below_ceiling(voxels.points, z_ref or 0.0, MAP_CEILING)
                rr.log("world/map", rr.Points3D(shown, colors=height_colors(shown[:, 2]), radii=MAP_VOXEL / 2))
                if len(path) > 1:
                    rr.log("world/path", rr.LineStrips3D([path], colors=[255, 140, 0]))
                log.info("map: %d points (%d scans), extent %s m", len(voxels.points), clouds,
                         [round(float(v), 1) for v in np.ptp(voxels.points, axis=0)] if len(voxels.points) else "-")
                t_logged, dirty = now, False

            if not clouds and now - t_start > 10 and now - t_rx > 10:
                log.warning("No SLAM scans yet — mapping only publishes while active. Check the uslam "
                            "log lines above, or start it with: python go2_client/cli_client.py slam mapping/start")
                t_rx = now
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        ctx.term()
        log.info("Mapping keeps running on the robot. 'slam mapping/stop' saves the map and "
                 "overwrites the robot's single saved map.")


if __name__ == "__main__":
    main()
