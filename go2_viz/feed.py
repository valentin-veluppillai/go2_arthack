"""Robot data → go2_viz inputs: python -m go2_viz.feed

Reads go2_yolo's pose detections (port 5557), turns the most prominent person
into the human/robot inputs of the co-creation spec and publishes them to
go2_viz (port 5561) about 10 times per second:

    humanBearing  box centre's offset from the image centre, through the camera's field of view
    distance      pinhole estimate from the torso length (shoulders → hips) and the box height,
                  the nearer of the two (both err on the far side when the person is cut off or bends)
    scale         share of the frame the box covers, 1 at GO2_FEED_SCALE_FULL
    confidence    mean keypoint confidence
    velocity      mean keypoint speed in box heights per second, minus jitter, 1 at GO2_FEED_VEL_FULL
    entropy       normalised Shannon entropy of the velocity over the last few seconds:
                  0 = steady (still or moving evenly), 1 = erratic
    trackHuman    true while a person is seen; headYaw / headPitch stay 0 (the Go2 has no separate head)

Without a person for GO2_FEED_LOST_S the inputs ease back to neutral
(distance 5 m, no movement, no confidence, scale 0, head straight ahead).
Camera motion (the robot turning) shows up as human movement.
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

FEED_HZ = 10
FEED_HFOV = float(os.getenv("GO2_FEED_HFOV", "120"))  # horizontal field of view of the front camera (°)
FEED_PERSON_H = float(os.getenv("GO2_FEED_PERSON_H", "1.7"))  # assumed body height (m)
FEED_TORSO = float(os.getenv("GO2_FEED_TORSO", "0.5"))  # assumed shoulder-to-hip length (m)
FEED_CONF = float(os.getenv("GO2_FEED_CONF", "0.5"))  # minimum person detection confidence
FEED_KPT_CONF = 0.5  # keypoints below this confidence are ignored for geometry and movement
FEED_SCALE_FULL = float(os.getenv("GO2_FEED_SCALE_FULL", "0.25"))  # box share of the frame for scale 1
FEED_VEL_FULL = float(os.getenv("GO2_FEED_VEL_FULL", "0.4"))  # box heights/s for velocity 1
FEED_VEL_NOISE = 0.05  # box heights/s of keypoint jitter on a person standing still
FEED_ENTROPY_S = float(os.getenv("GO2_FEED_ENTROPY_S", "3"))  # entropy window (s)
FEED_LOST_S = float(os.getenv("GO2_FEED_LOST_S", "1.0"))  # no person for this long → neutral
ENTROPY_BINS = 8
SMOOTH = 0.3  # share of a new velocity sample per update (keypoints are noisy)

NEUTRAL = {"distance": 5.0, "velocity": 0.0, "entropy": 0.0, "confidence": 0.0, "scale": 0.0,
           "headYaw": 0.0, "headPitch": 0.0, "trackHuman": False}
SHOULDERS, HIPS = (5, 6), (11, 12)  # COCO keypoint indices

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("go2_viz.feed")


def clamp(value: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


def pick_person(msg: dict) -> dict | None:
    """Largest confident person box, or None."""
    people = [d for d in msg["detections"] if d["cls"] == "person" and d["conf"] >= FEED_CONF]
    if not people:
        return None
    return max(people, key=lambda d: (d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1]))


class Feed:
    """Turns successive detections messages into go2_viz inputs."""

    def __init__(self) -> None:
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
        focal = (w / 2) / math.tan(math.radians(FEED_HFOV) / 2)  # px, square pixels assumed
        kpts = np.array(person.get("keypoints") or np.zeros((0, 3)), dtype=float).reshape(-1, 3)

        out = {"trackHuman": True, "headYaw": 0.0, "headPitch": 0.0}
        out["humanBearing"] = 0.0 - math.degrees(math.atan(((x1 + x2) / 2 - w / 2) / focal))  # + = left
        out["scale"] = clamp((x2 - x1) * box_h / (w * h) / FEED_SCALE_FULL)

        estimates = [FEED_PERSON_H * focal / box_h]
        if len(kpts) == 17 and (kpts[[*SHOULDERS, *HIPS], 2] >= FEED_KPT_CONF).all():
            torso = np.linalg.norm(kpts[list(SHOULDERS), :2].mean(0) - kpts[list(HIPS), :2].mean(0))
            if torso > 1:
                estimates.append(FEED_TORSO * focal / torso)
        out["distance"] = clamp(min(estimates), 0.3, 5.0)

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

    feed = Feed()
    inputs: dict = {}
    t_seen = 0.0  # monotonic time of the last frame with a person
    t_log = 0.0
    try:
        while True:
            t_next = time.monotonic() + 1 / FEED_HZ
            update = None
            # handle every detections message that arrived, so velocity sees each frame
            while sub.poll(max(0.0, t_next - time.monotonic()) * 1000):
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
                log.info("%s", "  ".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}"
                                         for k, v in inputs.items()))
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        pub.close(linger=0)
        ctx.term()


if __name__ == "__main__":
    main()
