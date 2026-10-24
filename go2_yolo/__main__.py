"""YOLO subscriber: python -m go2_yolo

Subscribes to the bridge's camera stream, runs YOLO on the newest frame,
shows the annotated feed in a window and republishes the results on its
own ZMQ PUB socket:

    [b"camera_yolo", annotated_jpeg]
    [b"detections",  json]   {"ts", "width", "height", "detections": [{"cls", "conf", "box", "keypoints"}]}

With a pose model (the default) every detection also carries "keypoints":
17 COCO body joints as [x, y, conf] in pixels; detection models leave it out.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time

import cv2
import numpy as np
import zmq
from ultralytics import YOLO

BRIDGE_HOST = os.getenv("GO2_BRIDGE_HOST", "localhost")
PUB_PORT = int(os.getenv("GO2_ZMQ_PUB_PORT", "5556"))
YOLO_PUB_PORT = int(os.getenv("GO2_YOLO_PUB_PORT", "5557"))
YOLO_MODEL = os.getenv("GO2_YOLO_MODEL", "yolo26n-pose.pt")
YOLO_CONF = float(os.getenv("GO2_YOLO_CONF", "0.4"))
YOLO_DEVICE = os.getenv("GO2_YOLO_DEVICE") or None  # e.g. "cpu", "mps", "0"
YOLO_SHOW = os.getenv("GO2_YOLO_SHOW", "1") == "1"

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("go2_yolo")


class LatestFrame:
    """Receives camera frames in a background thread and keeps only the newest,
    so a slow model never falls behind the stream."""

    def __init__(self, ctx: zmq.Context) -> None:
        self._ctx = ctx
        self._jpeg: bytes | None = None
        self._lock = threading.Lock()
        self._new = threading.Event()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="camera-sub")
        self._thread.start()

    def get(self, timeout: float) -> bytes | None:
        if not self._new.wait(timeout):
            return None
        with self._lock:
            self._new.clear()
            return self._jpeg

    def shutdown(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        sub = self._ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.RCVTIMEO, 500)
        sub.connect(f"tcp://{BRIDGE_HOST}:{PUB_PORT}")
        sub.subscribe(b"camera")
        while self._running:
            try:
                _topic, jpeg = sub.recv_multipart()
            except zmq.Again:
                continue
            with self._lock:
                self._jpeg = jpeg
                self._new.set()
        sub.close()


def to_detections(result) -> list[dict]:
    names = result.names
    detections = [
        {
            "cls": names[int(c)],
            "conf": round(float(p), 3),
            "box": [round(float(v), 1) for v in xyxy],  # x1, y1, x2, y2 in pixels
        }
        for xyxy, c, p in zip(result.boxes.xyxy, result.boxes.cls, result.boxes.conf)
    ]
    if result.keypoints is not None and result.keypoints.has_visible:
        for det, kpts in zip(detections, result.keypoints.data.tolist()):
            det["keypoints"] = [[round(x, 1), round(y, 1), round(c, 3)] for x, y, c in kpts]
    return detections


def main() -> None:
    log.info("Loading model %s", YOLO_MODEL)
    model = YOLO(YOLO_MODEL)

    ctx = zmq.Context()
    frames = LatestFrame(ctx)

    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{YOLO_PUB_PORT}")

    log.info("Subscribed to tcp://%s:%d, publishing on port %d", BRIDGE_HOST, PUB_PORT, YOLO_PUB_PORT)

    try:
        while True:
            jpeg = frames.get(timeout=5.0)
            if jpeg is None:
                log.warning("No camera frame for 5 s — is the bridge running?")
                continue

            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                log.warning("Could not decode frame (%d bytes)", len(jpeg))
                continue

            t0 = time.perf_counter()
            result = model(frame, conf=YOLO_CONF, device=YOLO_DEVICE, verbose=False)[0]
            infer_ms = (time.perf_counter() - t0) * 1000

            detections = to_detections(result)
            annotated = result.plot()
            cv2.putText(annotated, f"{infer_ms:.0f} ms", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            ok, out_jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                pub.send_multipart([b"camera_yolo", out_jpeg.tobytes()])
            pub.send_multipart([b"detections", json.dumps({
                "ts": time.time(),
                "width": frame.shape[1],
                "height": frame.shape[0],
                "detections": detections,
            }).encode()])

            if YOLO_SHOW:
                cv2.imshow("go2 yolo", annotated)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        frames.shutdown()
        pub.close()
        ctx.term()
        if YOLO_SHOW:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
