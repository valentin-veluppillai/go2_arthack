"""Live 3D map: python -m go2_map

Subscribes to the bridge's SLAM stream (port 5559), merges every world-frame
scan from the Go2's on-board SLAM (uslam_server) into one voxel map and shows
it live in a Rerun 3D viewer, together with the robot pose, its path and the
current scan. The map keeps everything seen so far, so the whole area around
the robot stays visible.

Moving objects (people walking past) would leave trails in that map. A coarse
occupancy grid removes them: every scan ray marks the space it passes through
as free, so cells that were only briefly occupied are cleared again, while
walls and furniture keep getting hit. Scan points that land in space seen
free before are shown separately as "moving".

Map points are also coloured from the front camera (bridge PUB port 5556): a
few times per second the newest frame is shrunk, blurred heavily (so pose
timing and calibration errors only shift soft gradients) and the map points in
view are projected into it with the SLAM pose. A coarse depth buffer, which
also holds the latest scan, keeps surfaces hidden behind others (or behind a
person) from taking their colour. Each point keeps a running average. Frames
are skipped while the robot turns fast or the pose is stale.

The cleaned map is also published once per second on ZMQ PUB port 5560 as
[b"occupancy", meta_json, float32 xyz, uint8 rgba] for other apps such as
go2_viz; alpha says how well a point's colour is known (0 = never seen).

uslam only publishes while it is mapping (or localized on a saved map), so by
default this sends "mapping/start" through the bridge at startup.
"""

from __future__ import annotations

import io
import json
import logging
import math
import os
import time

import numpy as np
import rerun as rr
import zmq
from PIL import Image, ImageFilter

BRIDGE_HOST = os.getenv("GO2_BRIDGE_HOST", "localhost")
CMD_PORT = int(os.getenv("GO2_ZMQ_CMD_PORT", "5555"))
SLAM_PORT = int(os.getenv("GO2_ZMQ_SLAM_PORT", "5559"))
CAMERA_PORT = int(os.getenv("GO2_ZMQ_PUB_PORT", "5556"))
MAP_PUB_PORT = int(os.getenv("GO2_MAP_PUB_PORT", "5560"))  # cleaned map for other apps
MAP_VOXEL = float(os.getenv("GO2_MAP_VOXEL", "0.05"))  # metres per map point
# hide points more than this far above the robot's start height, so indoor ceilings
# don't cover the map (0 = show everything)
MAP_CEILING = float(os.getenv("GO2_MAP_CEILING", "1.5"))
MAP_START = os.getenv("GO2_MAP_START", "1") == "1"  # send mapping/start at startup
MAP_WEB = os.getenv("GO2_MAP_WEB", "0") == "1"  # browser viewer instead of the native one
MAP_SAVE = os.getenv("GO2_MAP_SAVE", "")  # write a .rrd recording instead of showing it
MAP_VIEWER_MEMORY = os.getenv("GO2_MAP_VIEWER_MEMORY", "1GB")  # memory cap of the live viewer
# The live viewer only keeps the latest map, scan and pose (no timeline), so its memory stays
# flat over hours; a .rrd recording (GO2_MAP_SAVE) keeps the full history.
LOG_HISTORY = bool(MAP_SAVE)
# .ply written on exit (default: go2_map_<timestamp>.ply in the current directory, "0" = off)
MAP_PLY = os.getenv("GO2_MAP_PLY", "")
MAP_FILTER = os.getenv("GO2_MAP_FILTER", "1") == "1"  # remove moving objects from the map
MAP_FILTER_CELL = float(os.getenv("GO2_MAP_FILTER_CELL", "0.1"))  # occupancy cell size, metres
MAP_LOG_HZ = 1.0  # full-map refresh rate in the viewer
# LiDAR position on the body (URDF radar_joint), the origin of every scan ray
LIDAR_OFFSET = np.array([0.28945, 0.0, -0.046825])

# colours from the front camera
MAP_COLOR = os.getenv("GO2_MAP_COLOR", "1") == "1"
MAP_COLOR_HZ = float(os.getenv("GO2_MAP_COLOR_HZ", "3"))  # camera frames used per second
MAP_COLOR_BLUR = float(os.getenv("GO2_MAP_COLOR_BLUR", "0.03"))  # blur sigma, fraction of the image width
MAP_COLOR_RANGE = float(os.getenv("GO2_MAP_COLOR_RANGE", "5"))  # m, farther points are not coloured
CAMERA_HFOV = float(os.getenv("GO2_CAMERA_HFOV", "120"))  # horizontal field of view of the front camera (°)
CAMERA_PITCH = float(os.getenv("GO2_CAMERA_PITCH", "0"))  # ° the front camera looks down
CAMERA_POS = np.array([0.32715, 0.0, 0.04297])  # front camera in the body frame (URDF)
COLOR_WIDTH = 160  # px, frames are shrunk to this width before blurring and sampling
COLOR_MAX_TURN = 0.6  # rad/s, frames taken while turning faster are skipped (blur, pose lag)
COLOR_POSE_AGE = 0.3  # s, frames are skipped when the newest pose is older
COLOR_OCCLUSION = 0.25  # m behind the nearest surface in a depth-buffer cell that still counts as visible
COLOR_MIN_RATE = 0.1  # share of each new sample in a point's colour at least, so colours follow lighting
COLOR_FULL_SEEN = 3  # observations for full alpha

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


def voxel_keys(pts: np.ndarray, size: float) -> np.ndarray:
    """Pack voxel indices into one int64 per point (21 bits per axis)."""
    idx = np.floor(pts / size).astype(np.int64) + (1 << 20)  # ±52 km at 5 cm
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


def rotate(q: list[float], v: np.ndarray) -> np.ndarray:
    """Rotate vector v by quaternion q = [x, y, z, w]."""
    u, w = np.asarray(q[:3]), q[3]
    return v + 2 * w * np.cross(u, v) + 2 * np.cross(u, np.cross(u, v))


class VoxelMap:
    """Keeps one point per voxel; the first point seen in a voxel stays. Each voxel also has a
    camera colour (0–1) and how often it was seen by the camera."""

    def __init__(self, voxel: float) -> None:
        self.voxel = voxel
        self.keys = np.empty(0, dtype=np.int64)
        self.points = np.empty((0, 3), dtype=np.float32)
        self.rgb = np.empty((0, 3), dtype=np.float32)
        self.seen = np.empty(0, dtype=np.float32)

    def add(self, pts: np.ndarray) -> None:
        keys = np.concatenate([self.keys, voxel_keys(pts, self.voxel)])
        points = np.concatenate([self.points, pts])
        rgb = np.concatenate([self.rgb, np.zeros((len(pts), 3), dtype=np.float32)])
        seen = np.concatenate([self.seen, np.zeros(len(pts), dtype=np.float32)])
        self.keys, first = np.unique(keys, return_index=True)  # existing voxels come first
        self.points, self.rgb, self.seen = points[first], rgb[first], seen[first]

    def colors(self, sel: np.ndarray) -> np.ndarray:
        """uint8 colours of the selected voxels: camera colour where seen, height colour elsewhere."""
        colors = height_colors(self.points[sel, 2])
        seen = self.seen[sel] > 0
        colors[seen] = (self.rgb[sel][seen] * 255).round().astype(np.uint8)
        return colors

    def rgba(self, sel: np.ndarray) -> np.ndarray:
        """uint8 camera colour of the selected voxels, alpha = how well it is known."""
        alpha = np.minimum(self.seen[sel] / COLOR_FULL_SEEN, 1.0)
        return (np.concatenate([self.rgb[sel], alpha[:, None]], axis=1) * 255).round().astype(np.uint8)


def turn_rate(a: dict, b: dict) -> float:
    """Rotation speed (rad/s) between two odometry messages."""
    dot = abs(float(np.dot(a["orientation"], b["orientation"])))
    dt = b["stamp"] - a["stamp"]
    return 2 * math.acos(min(dot, 1.0)) / dt if dt > 0 else 0.0


def to_camera(points: np.ndarray, odom: dict) -> np.ndarray:
    """World points → camera frame: x right, y down, z forward (m)."""
    qx, qy, qz, qw = odom["orientation"]
    c = rotate([-qx, -qy, -qz, qw], points - np.asarray(odom["position"])) - CAMERA_POS
    pitch = math.radians(CAMERA_PITCH)
    forward = c[:, 0] * math.cos(pitch) - c[:, 2] * math.sin(pitch)
    up = c[:, 0] * math.sin(pitch) + c[:, 2] * math.cos(pitch)
    return np.stack([-c[:, 1], -up, forward], axis=1)


def colorize(voxels: VoxelMap, jpeg: bytes, odom: dict, scan: np.ndarray) -> int:
    """Blend one camera frame into the colours of the map points it sees; returns how many."""
    image = Image.open(io.BytesIO(jpeg)).convert("RGB")
    width, height = COLOR_WIDTH, max(1, round(COLOR_WIDTH * image.height / image.width))
    image = image.resize((width, height), Image.BILINEAR)
    image = image.filter(ImageFilter.GaussianBlur(MAP_COLOR_BLUR * width))
    pixels = np.asarray(image, dtype=np.float32) / 255
    focal = (width / 2) / math.tan(math.radians(CAMERA_HFOV) / 2)

    def project(points: np.ndarray):
        cam = to_camera(points, odom)
        z = cam[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = width / 2 + focal * cam[:, 0] / z
            v = height / 2 + focal * cam[:, 1] / z
        ok = (z > 0.1) & (z < MAP_COLOR_RANGE) & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        return u.astype(int), v.astype(int), z, ok

    near = np.flatnonzero(np.linalg.norm(voxels.points[:, :2] - np.asarray(odom["position"][:2]), axis=1)
                          < MAP_COLOR_RANGE)
    u, v, z, ok = project(voxels.points[near])
    near, u, v, z = near[ok], u[ok], v[ok], z[ok]
    if not len(near):
        return 0

    # depth buffer at half the resolution; the latest scan adds what the map leaves out (people)
    depth = np.full((height // 2 + 1, width // 2 + 1), np.inf)
    np.minimum.at(depth, (v // 2, u // 2), z)
    su, sv, sz, sok = project(scan)
    np.minimum.at(depth, (sv[sok] // 2, su[sok] // 2), sz[sok])
    visible = z <= depth[v // 2, u // 2] + COLOR_OCCLUSION
    near, u, v = near[visible], u[visible], v[visible]

    rate = np.maximum(1 / (voxels.seen[near] + 1), COLOR_MIN_RATE)[:, None]
    voxels.rgb[near] += (pixels[v, u] - voxels.rgb[near]) * rate
    voxels.seen[near] += 1
    return len(near)


class OccupancyGrid:
    """Coarse log-odds occupancy grid: hits raise a cell, rays passing through lower it.

    A person walking past hits a cell once or twice; later rays pass through
    the then-empty cell and clear it, so the trail disappears. Static surfaces
    are hit over and over and stay.
    """

    HIT, MISS, LOW, HIGH = 0.85, -0.4, -2.0, 3.5

    def __init__(self, cell: float) -> None:
        self.cell = cell
        self.keys = np.empty(0, dtype=np.int64)  # sorted
        self.odds = np.empty(0, dtype=np.float32)

    def odds_at(self, pts: np.ndarray) -> np.ndarray:
        """Log-odds of the cell each point falls in (0 = never observed)."""
        if not len(self.keys):
            return np.zeros(len(pts), dtype=np.float32)
        keys = voxel_keys(pts, self.cell)
        idx = np.minimum(np.searchsorted(self.keys, keys), len(self.keys) - 1)
        return np.where(self.keys[idx] == keys, self.odds[idx], 0.0)

    def _add(self, keys: np.ndarray, delta: float) -> None:
        """Add delta to the cells in keys (unique), creating unseen ones."""
        idx = np.searchsorted(self.keys, keys)
        found = idx < len(self.keys)
        found[found] = self.keys[idx[found]] == keys[found]
        self.odds[idx[found]] = np.clip(self.odds[idx[found]] + delta, self.LOW, self.HIGH)
        new = keys[~found]
        if len(new):
            keys_all = np.concatenate([self.keys, new])
            odds_all = np.concatenate([self.odds, np.full(len(new), delta, dtype=np.float32)])
            order = np.argsort(keys_all)
            self.keys, self.odds = keys_all[order], odds_all[order]

    def update(self, origin: np.ndarray, scan: np.ndarray, min_z: float) -> None:
        """Integrate one scan taken from origin; space below min_z (the floor) is never cleared."""
        hits = np.unique(voxel_keys(scan, self.cell))
        d = scan - origin
        r = np.linalg.norm(d, axis=1)
        keep = (r > 0.1) & (r < 15.0)
        d, r = d[keep] / r[keep, None], r[keep]
        if not len(r):
            return
        steps = np.arange(self.cell / 2, r.max(), self.cell / 2)
        # stop two cells short of each hit so grazing rays don't eat into the surface
        along = steps[None, :] < (r[:, None] - 2 * self.cell)
        samples = (origin + d[:, None, :] * steps[None, :, None])[along]
        samples = samples[samples[:, 2] > min_z]
        misses = np.setdiff1d(np.unique(voxel_keys(samples, self.cell)), hits, assume_unique=True)
        self._add(misses, self.MISS)
        self._add(hits, self.HIT)


def below_ceiling(points: np.ndarray, z_ref: float, ceiling: float) -> np.ndarray:
    return points if ceiling <= 0 else points[points[:, 2] < z_ref + ceiling]


def height_colors(z: np.ndarray) -> np.ndarray:
    """Blue (low) → green → yellow → red (high), robust to outliers."""
    lo, hi = np.percentile(z, [2, 98]) if len(z) else (0.0, 1.0)
    t = np.clip((z - lo) / max(hi - lo, 1e-3), 0, 1)
    anchors = np.array([[40, 70, 220], [40, 190, 220], [80, 200, 80], [240, 220, 60], [230, 60, 40]])
    pos = np.linspace(0, 1, len(anchors))
    return np.stack([np.interp(t, pos, anchors[:, c]) for c in range(3)], axis=1).astype(np.uint8)


def save_ply(path: str, points: np.ndarray, colors: np.ndarray) -> None:
    """Binary little-endian PLY with x, y, z (float) and red, green, blue (uchar)."""
    rec = np.zeros(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                       ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec["x"], rec["y"], rec["z"] = points.T
    rec["red"], rec["green"], rec["blue"] = colors.T
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(rec)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(rec.tobytes())


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
        rr.serve_web_viewer(connect_to=rr.serve_grpc(server_memory_limit=MAP_VIEWER_MEMORY))
    else:
        # the viewer's built-in gRPC server buffers messages for viewers that connect later (1 GiB by
        # default); ours is connected from the start, so a small buffer is enough
        rr.spawn(memory_limit=MAP_VIEWER_MEMORY, server_memory_limit="64MB")
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    # Go2 body ≈ 0.70 × 0.31 × 0.40 m, with a heading arrow
    rr.log("world/robot/body", rr.Boxes3D(half_sizes=[0.35, 0.155, 0.2], colors=[255, 140, 0]), static=True)
    rr.log("world/robot/heading", rr.Arrows3D(vectors=[[0.6, 0, 0]], colors=[255, 140, 0]), static=True)


def show(entity_path: str, entity) -> None:
    """Log to the viewer; without history each update replaces the previous one."""
    rr.log(entity_path, entity, static=not LOG_HISTORY)


def main() -> None:
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(f"tcp://{BRIDGE_HOST}:{SLAM_PORT}")
    for topic in (b"slam_cloud", b"slam_odom", b"slam_log"):
        sub.subscribe(topic)
    log.info("Subscribed to tcp://%s:%d", BRIDGE_HOST, SLAM_PORT)
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    cam = None
    if MAP_COLOR:
        cam = ctx.socket(zmq.SUB)
        cam.setsockopt(zmq.RCVHWM, 2)  # only the newest frame matters
        cam.connect(f"tcp://{BRIDGE_HOST}:{CAMERA_PORT}")
        cam.subscribe(b"camera")
        poller.register(cam, zmq.POLLIN)
        log.info("Colouring the map from the camera on tcp://%s:%d", BRIDGE_HOST, CAMERA_PORT)

    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 2)  # a stalled subscriber gets the newest maps, not a growing backlog
    pub.bind(f"tcp://*:{MAP_PUB_PORT}")

    init_viewer()
    if MAP_START:
        time.sleep(0.3)  # let the SUB socket connect so the ack isn't missed
        send_slam_command(ctx, "mapping/start")

    voxels = VoxelMap(MAP_VOXEL)
    grid = OccupancyGrid(MAP_FILTER_CELL)

    def occupied() -> np.ndarray:
        """Mask of the whole map with moving objects removed (no ceiling cut)."""
        return grid.odds_at(voxels.points) >= 0 if MAP_FILTER else np.ones(len(voxels.points), dtype=bool)
    odom: dict | None = None
    t_odom = 0.0  # time.time() of the newest pose
    turning = 0.0  # rad/s between the last two poses
    scan = np.empty((0, 3), dtype=np.float32)  # latest scan
    jpeg: bytes | None = None  # newest camera frame, not used yet
    t_colored = 0.0
    colored = 0  # points coloured since the last log line
    moving = np.empty((0, 3), dtype=np.float32)
    path: list[list[float]] = []
    z_ref: float | None = None  # robot height at the first pose, reference for the ceiling cut
    t_start = t_logged = t_rx = time.time()
    clouds = 0
    dirty = False

    try:
        while True:
            events = dict(poller.poll(1000))  # short, so Ctrl+C is handled promptly
            topic = None
            if sub in events:
                topic, *frames = sub.recv_multipart()
            if cam is not None and cam in events:
                while cam.poll(0):
                    _cam_topic, jpeg = cam.recv_multipart()

            if topic == b"slam_cloud":
                t_rx = time.time()
                scan = cloud_to_xyz(json.loads(frames[0]), frames[1])
                if MAP_FILTER and odom is not None:
                    # hits in space that was seen free before = something moved there
                    moving = scan[grid.odds_at(scan) < -0.5]
                    show("world/moving", rr.Points3D(moving, colors=[255, 0, 200], radii=0.05))
                    origin = np.asarray(odom["position"]) + rotate(odom["orientation"], LIDAR_OFFSET)
                    floor_z = np.percentile(voxels.points[:, 2], 2) if len(voxels.points) > 1000 else -np.inf
                    grid.update(origin, scan, floor_z + 0.2)
                voxels.add(scan)
                show("world/scan", rr.Points3D(below_ceiling(scan, z_ref or 0.0, MAP_CEILING),
                                               colors=[255, 255, 255], radii=0.02))
                clouds += 1
                dirty = True
            elif topic == b"slam_odom":
                previous, odom, t_odom = odom, json.loads(frames[0]), time.time()
                if previous is not None:
                    turning = turn_rate(previous, odom)
                if z_ref is None:
                    z_ref = odom["position"][2]
                show("world/robot", rr.Transform3D(translation=odom["position"],
                                                   rotation=rr.Quaternion(xyzw=odom["orientation"])))
                if not path or np.linalg.norm(np.subtract(odom["position"], path[-1])) > 0.05:
                    path.append(odom["position"])
                    dirty = True
            elif topic == b"slam_log":
                log.info("uslam: %s", frames[0].decode(errors="replace"))

            now = time.time()
            if (jpeg is not None and odom is not None and len(voxels.points)
                    and now - t_colored >= 1.0 / MAP_COLOR_HZ):
                # a frame taken while turning fast is blurred and lags the pose: skip it
                if now - t_odom < COLOR_POSE_AGE and turning < COLOR_MAX_TURN:
                    colored += colorize(voxels, jpeg, odom, scan)
                    t_colored, dirty = now, True
                jpeg = None

            if dirty and now - t_logged >= 1.0 / MAP_LOG_HZ:
                keep = occupied()
                shown = keep.copy()  # drop cells cleared by later rays, and the ceiling
                if MAP_CEILING > 0:
                    shown &= voxels.points[:, 2] < (z_ref or 0.0) + MAP_CEILING
                show("world/map", rr.Points3D(voxels.points[shown], colors=voxels.colors(shown), radii=MAP_VOXEL / 2))
                if len(path) > 1:
                    show("world/path", rr.LineStrips3D([path], colors=[255, 140, 0]))
                points = voxels.points[keep]
                pub.send_multipart([b"occupancy", json.dumps({
                    "stamp": now,
                    "count": len(points),
                    "voxel": MAP_VOXEL,
                    "z_ref": z_ref,
                    "robot": {"position": odom["position"], "orientation": odom["orientation"]} if odom else None,
                }).encode(), points.astype("<f4").tobytes(), voxels.rgba(keep).tobytes()])
                log.info("map: %d points shown (%d scans), extent %s m, %d moving points in the last scan, "
                         "%d coloured from the camera (%d points)",
                         int(shown.sum()), clouds,
                         [round(float(v), 1) for v in np.ptp(voxels.points, axis=0)] if len(voxels.points) else "-",
                         len(moving), int((voxels.seen > 0).sum()), colored)
                t_logged, dirty, colored = now, False, 0

            if not clouds and now - t_start > 10 and now - t_rx > 10:
                log.warning("No SLAM scans yet — mapping only publishes while active. Check the uslam "
                            "log lines above, or start it with: python go2_client/cli_client.py slam mapping/start")
                t_rx = now
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        if cam is not None:
            cam.close()
        pub.close()
        ctx.term()
        if MAP_PLY != "0" and len(voxels.points):
            keep = occupied()
            points = voxels.points[keep]
            path_ply = MAP_PLY or time.strftime("go2_map_%Y%m%d_%H%M%S.ply")
            save_ply(path_ply, points, voxels.colors(keep))
            log.info("Saved %d map points to %s", len(points), os.path.abspath(path_ply))
        log.info("Mapping keeps running on the robot. 'slam mapping/stop' saves the map and "
                 "overwrites the robot's single saved map.")


if __name__ == "__main__":
    main()
