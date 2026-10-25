"""Robot data → go2_viz inputs: python -m go2_viz.feed

Reads go2_yolo's pose detections (port 5557) and the bridge's SLAM stream
(port 5559), turns the most prominent person into the human/robot inputs of
the co-creation spec and publishes them to go2_viz (port 5561) about 10 times
per second:

    humanBearing  box centre's offset from the image centre, through the camera's field of view
    distance      LiDAR: the nearest cluster of scan points in the person's direction (the middle
                  half of the box, as seen from the camera), measured from the body centre, unless it
                  lies beyond twice the camera estimate (then it is the wall behind the person).
                  Fallback without LiDAR points: pinhole estimate from the torso length
                  (shoulders → hips) and the box height, the nearer of the two (both err on the
                  far side when the person is cut off or bends)
    scale         share of the frame the box covers, 1 at GO2_FEED_SCALE_FULL
    confidence    mean keypoint confidence
    velocity      mean keypoint speed in box heights per second, minus jitter, 1 at GO2_FEED_VEL_FULL
    entropy       normalised Shannon entropy of the velocity over the last few seconds:
                  0 = steady (still or moving evenly), 1 = erratic
    trackHuman    true while a person is seen; headYaw / headPitch stay 0 (the Go2 has no separate head)
    boxLeft, boxRight, boxTop, boxBottom
                  the person's bounding box as angles seen from the camera (°, + = left / up, elevation
                  from the horizontal); go2_viz shapes the human-channel vignette with it

Without a person for GO2_FEED_LOST_S the inputs ease back to neutral
(distance 5 m, no movement, no confidence, scale 0, head straight ahead).
Camera motion (the robot turning) shows up as human movement.

The SLAM scans are in the uslam world frame and are moved into the body frame
with uslam's own pose, so they only arrive while uslam is mapping or
localized (python -m go2_map starts mapping).
"""

from __future__ import annotations

import collections
import json
import logging
import math
import os
import time

import numpy as np
import zmq

YOLO_HOST = os.getenv("GO2_YOLO_HOST", os.getenv("GO2_BRIDGE_HOST", "localhost"))
YOLO_PUB_PORT = int(os.getenv("GO2_YOLO_PUB_PORT", "5557"))
VIZ_HOST = os.getenv("GO2_VIZ_PARAM_HOST", "localhost")
VIZ_PARAM_PORT = int(os.getenv("GO2_VIZ_PARAM_PORT", "5561"))
BRIDGE_HOST = os.getenv("GO2_BRIDGE_HOST", "localhost")
SLAM_PORT = int(os.getenv("GO2_ZMQ_SLAM_PORT", "5559"))

FEED_HZ = 10
# front camera model, shared with go2_map (fitted by python -m go2_map.calibrate; same defaults)
CAMERA_MODEL = os.getenv("GO2_CAMERA_MODEL", "fisheye")  # "pinhole" or "fisheye" (equidistant)
CAMERA_HFOV = float(os.getenv("GO2_CAMERA_HFOV", "95.5"))  # horizontal field of view of the front camera (°)
CAMERA_PITCH = float(os.getenv("GO2_CAMERA_PITCH", "-1.8"))  # ° the front camera looks down
CAMERA_YAW = float(os.getenv("GO2_CAMERA_YAW", "3.1"))  # ° the camera looks left of the body's heading
FEED_PERSON_H = float(os.getenv("GO2_FEED_PERSON_H", "1.7"))  # assumed body height (m)
FEED_TORSO = float(os.getenv("GO2_FEED_TORSO", "0.5"))  # assumed shoulder-to-hip length (m)
FEED_CONF = float(os.getenv("GO2_FEED_CONF", "0.5"))  # minimum person detection confidence
FEED_KPT_CONF = 0.5  # keypoints below this confidence are ignored for geometry and movement
FEED_SCALE_FULL = float(os.getenv("GO2_FEED_SCALE_FULL", "0.25"))  # box share of the frame for scale 1
FEED_VEL_FULL = float(os.getenv("GO2_FEED_VEL_FULL", "0.4"))  # box heights/s for velocity 1
FEED_VEL_NOISE = 0.05  # box heights/s of keypoint jitter on a person standing still
FEED_ENTROPY_S = float(os.getenv("GO2_FEED_ENTROPY_S", "3"))  # entropy window (s)
FEED_LOST_S = float(os.getenv("GO2_FEED_LOST_S", "1.0"))  # no person for this long → neutral
FEED_LIDAR = os.getenv("GO2_FEED_LIDAR", "1") == "1"  # distance from the SLAM scans when possible
FEED_LIDAR_WINDOW_S = float(os.getenv("GO2_FEED_LIDAR_WINDOW_S", "0.5"))  # scans merged (the L1 is sparse)
FEED_LIDAR_MIN_POINTS = int(os.getenv("GO2_FEED_LIDAR_MIN_POINTS", "3"))  # smallest cluster taken as the person
LIDAR_CLUSTER_DEPTH = 0.4  # m, depth of a person's cluster along the ray
LIDAR_MIN_HEIGHT = 0.5  # m, vertical extent a cluster needs to count as a person
LIDAR_VOXEL = 0.05  # m; merged scans repeat static points, so clusters count distinct voxels
LIDAR_BAND = (-0.15, 1.5)  # m, body-frame heights searched: above the floor (≈ -0.3), up to head height
LIDAR_MIN_SECTOR = 3.0  # °, half-width of the searched sector at least
# a LiDAR cluster farther than this × the camera estimate (+0.3 m) is something behind the person: the
# L1 sees little of a close person. Relative, because the camera estimate is rough (uncalibrated FOV).
LIDAR_BEYOND_CAMERA = 2.0
CAMERA_POS = np.array([0.32715, 0.0, 0.04297])  # front camera in the body frame (URDF)
ENTROPY_BINS = 8
SMOOTH = 0.3  # share of a new velocity sample per update (keypoints are noisy)

NEUTRAL = {"distance": 5.0, "velocity": 0.0, "entropy": 0.0, "confidence": 0.0, "scale": 0.0,
           "headYaw": 0.0, "headPitch": 0.0, "trackHuman": False}
SHOULDERS, HIPS = (5, 6), (11, 12)  # COCO keypoint indices

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("go2_viz.feed")


def clamp(value: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


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


def rotate(q, v: np.ndarray) -> np.ndarray:
    """Rotate vectors v (..., 3) by quaternion q = [x, y, z, w]."""
    u, w = np.asarray(q[:3]), q[3]
    return v + 2 * w * np.cross(u, v) + 2 * np.cross(u, np.cross(u, v))


class LidarRange:
    """Recent SLAM scans in the world frame plus the latest pose; measures the range in a direction."""

    def __init__(self) -> None:
        self.scans: collections.deque[tuple[float, np.ndarray]] = collections.deque()  # (time, world xyz)
        self.odom: dict | None = None

    def add_cloud(self, meta: dict, data: bytes) -> None:
        now = time.monotonic()
        self.scans.append((now, cloud_to_xyz(meta, data)))
        while self.scans and self.scans[0][0] < now - FEED_LIDAR_WINDOW_S:
            self.scans.popleft()

    def body_points(self) -> np.ndarray:
        """Recent scan points in the body frame (x forward, y left, z up)."""
        if self.odom is None or not self.scans or self.scans[-1][0] < time.monotonic() - FEED_LIDAR_WINDOW_S:
            return np.empty((0, 3))
        world = np.concatenate([pts for _, pts in self.scans]).astype(float)
        qx, qy, qz, qw = self.odom["orientation"]
        return rotate([-qx, -qy, -qz, qw], world - np.asarray(self.odom["position"]))

    def distance(self, left: float, right: float) -> float | None:
        """Horizontal range of the nearest point cluster between two bearings (°, + = left, seen
        from the camera), or None if there is none."""
        pts = self.body_points()
        pts = pts[(pts[:, 2] > LIDAR_BAND[0]) & (pts[:, 2] < LIDAR_BAND[1])]
        rel = pts - CAMERA_POS
        bearing = np.degrees(np.arctan2(rel[:, 1], rel[:, 0]))
        pts = pts[(rel[:, 0] > 0) & (bearing >= right) & (bearing <= left)]
        pts = np.unique(np.round(pts / LIDAR_VOXEL), axis=0) * LIDAR_VOXEL
        r = np.hypot(pts[:, 0], pts[:, 1])
        order = np.argsort(r)
        r, z = r[order], pts[order, 2]
        ends = np.searchsorted(r, r + LIDAR_CLUSTER_DEPTH)
        # nearest range with enough points behind it within a person's depth, spanning a person's
        # height (skips stray returns and low clutter like the floor's edge)
        for start in np.flatnonzero(ends - np.arange(len(r)) >= FEED_LIDAR_MIN_POINTS):
            zs = z[start:ends[start]]
            if zs.max() - zs.min() >= LIDAR_MIN_HEIGHT:
                return float(np.median(r[start:ends[start]]))
        return None


def pick_person(msg: dict) -> dict | None:
    """Largest confident person box, or None."""
    people = [d for d in msg["detections"] if d["cls"] == "person" and d["conf"] >= FEED_CONF]
    if not people:
        return None
    return max(people, key=lambda d: (d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1]))


class Feed:
    """Turns successive detections messages into go2_viz inputs."""

    def __init__(self, lidar: LidarRange | None = None) -> None:
        self.lidar = lidar
        self.distance_source = "-"
        self.prev: tuple[float, np.ndarray] | None = None  # ts, keypoints of the last frame with a person
        self.velocity = 0.0
        self.history: collections.deque[tuple[float, float]] = collections.deque()  # (time, velocity)

    def update(self, msg: dict) -> dict | None:
        """Inputs for one detections message, or None if it shows no person."""
        person = pick_person(msg)
        if person is None:
            return None
        ts, w, h = msg["ts"], msg["width"], msg["height"]
        x1, y1, x2, y2 = person["box"]
        box_h = max(y2 - y1, 1.0)
        # px per unit of tan(angle) (pinhole) or per radian (fisheye); square pixels assumed
        if CAMERA_MODEL == "fisheye":
            focal = (w / 2) / math.radians(CAMERA_HFOV / 2)
        else:
            focal = (w / 2) / math.tan(math.radians(CAMERA_HFOV) / 2)
        kpts = np.array(person.get("keypoints") or np.zeros((0, 3)), dtype=float).reshape(-1, 3)

        def angle(offset: float) -> float:  # ° from the optical axis for a pixel offset from the centre
            return math.degrees(offset / focal if CAMERA_MODEL == "fisheye" else math.atan(offset / focal))

        def bearing(x: float) -> float:  # ° from the body's heading, + = left
            return 0.0 - angle(x - w / 2) + CAMERA_YAW

        def elevation(y: float) -> float:  # ° above the horizontal (the camera looks down by CAMERA_PITCH)
            return 0.0 - angle(y - h / 2) - CAMERA_PITCH

        out = {"trackHuman": True, "headYaw": 0.0, "headPitch": 0.0}
        out["humanBearing"] = bearing((x1 + x2) / 2)
        out.update(boxLeft=bearing(x1), boxRight=bearing(x2), boxTop=elevation(y1), boxBottom=elevation(y2))
        out["scale"] = clamp((x2 - x1) * box_h / (w * h) / FEED_SCALE_FULL)

        estimates = [FEED_PERSON_H * focal / box_h]
        if len(kpts) == 17 and (kpts[[*SHOULDERS, *HIPS], 2] >= FEED_KPT_CONF).all():
            torso = np.linalg.norm(kpts[list(SHOULDERS), :2].mean(0) - kpts[list(HIPS), :2].mean(0))
            if torso > 1:
                estimates.append(FEED_TORSO * focal / torso)
        measured = None
        if self.lidar is not None:
            half = max((bearing(x1) - bearing(x2)) / 4, LIDAR_MIN_SECTOR)  # middle half of the box
            measured = self.lidar.distance(out["humanBearing"] + half, out["humanBearing"] - half)
        if measured is not None and measured > LIDAR_BEYOND_CAMERA * min(estimates) + 0.3:
            measured = None
        self.distance_source = "lidar" if measured is not None else "camera"
        out["distance"] = clamp(measured if measured is not None else min(estimates), 0.3, 5.0)

        if len(kpts):
            out["confidence"] = clamp(float(kpts[:, 2].mean()))
            if self.prev is not None and len(self.prev[1]) == len(kpts) and 0 < ts - self.prev[0] < FEED_LOST_S:
                both = (kpts[:, 2] >= FEED_KPT_CONF) & (self.prev[1][:, 2] >= FEED_KPT_CONF)
                if both.any():
                    step = np.linalg.norm(kpts[both, :2] - self.prev[1][both, :2], axis=1).mean()
                    speed = step / box_h / (ts - self.prev[0])  # box heights per second
                    sample = clamp((speed - FEED_VEL_NOISE) / FEED_VEL_FULL)
                    self.velocity += SMOOTH * (sample - self.velocity)
            self.prev = (ts, kpts)
        else:
            out["confidence"] = clamp(person["conf"])  # detection model: no keypoints
        out["velocity"] = self.velocity
        out["entropy"] = self._entropy(time.time())  # local clock: go2_yolo may run on another machine
        return out

    def lost(self, now: float) -> dict:
        """Inputs while no person is seen: movement decays, the rest goes neutral."""
        self.prev = None
        self.velocity = 0.0
        return {**NEUTRAL, "entropy": self._entropy(now)}

    def _entropy(self, ts: float) -> float:
        """Normalised entropy of the velocity distribution over the last FEED_ENTROPY_S seconds."""
        self.history.append((ts, self.velocity))
        while self.history and self.history[0][0] < ts - FEED_ENTROPY_S:
            self.history.popleft()
        counts = np.bincount(np.minimum((np.array([v for _, v in self.history]) * ENTROPY_BINS).astype(int),
                                        ENTROPY_BINS - 1), minlength=ENTROPY_BINS)
        p = counts[counts > 0] / counts.sum()
        return max(0.0, float(-(p * np.log(p)).sum() / math.log(ENTROPY_BINS)))


def main() -> None:
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(f"tcp://{YOLO_HOST}:{YOLO_PUB_PORT}")
    sub.subscribe(b"detections")
    pub = ctx.socket(zmq.PUB)
    pub.connect(f"tcp://{VIZ_HOST}:{VIZ_PARAM_PORT}")
    log.info("Detections from tcp://%s:%d → inputs to tcp://%s:%d",
             YOLO_HOST, YOLO_PUB_PORT, VIZ_HOST, VIZ_PARAM_PORT)
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    lidar = None
    if FEED_LIDAR:
        lidar = LidarRange()
        slam = ctx.socket(zmq.SUB)
        slam.setsockopt(zmq.RCVHWM, 10)
        slam.connect(f"tcp://{BRIDGE_HOST}:{SLAM_PORT}")
        slam.subscribe(b"slam_cloud")
        slam.subscribe(b"slam_odom")
        poller.register(slam, zmq.POLLIN)
        log.info("LiDAR distance from SLAM scans on tcp://%s:%d", BRIDGE_HOST, SLAM_PORT)

    feed = Feed(lidar)
    inputs: dict = {}
    t_seen = 0.0  # monotonic time of the last frame with a person
    t_log = 0.0
    try:
        while True:
            t_next = time.monotonic() + 1 / FEED_HZ
            update = None
            # handle every message that arrived, so velocity sees each frame
            while ready := dict(poller.poll(max(0.0, t_next - time.monotonic()) * 1000)):
                if lidar is not None and slam in ready:
                    topic, *frames = slam.recv_multipart()
                    if topic == b"slam_cloud":
                        lidar.add_cloud(json.loads(frames[0]), frames[1])
                    else:
                        lidar.odom = json.loads(frames[0])
                if sub in ready:
                    _topic, payload = sub.recv_multipart()
                    result = feed.update(json.loads(payload))
                    if result is not None:
                        update, t_seen = result, time.monotonic()
            now = time.monotonic()
            if update is not None:
                inputs = update
            elif now - t_seen > FEED_LOST_S:
                inputs = feed.lost(time.time())
            if inputs:
                pub.send_multipart([b"viz", json.dumps(inputs).encode()])
            if now - t_log > 1.0:
                t_log = now
                log.info("%s  (distance from %s)", "  ".join(
                    f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}" for k, v in inputs.items()),
                    feed.distance_source if inputs.get("trackHuman") else "-")
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        if lidar is not None:
            slam.close()
        pub.close(linger=0)
        ctx.term()


if __name__ == "__main__":
    main()
