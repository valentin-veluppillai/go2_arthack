"""LiDAR subscriber: python -m go2_lidar

Subscribes to the bridge's LiDAR stream and logs how many points arrive.
Messages are three-part: [b"lidar", meta_json, point_data], where meta holds
the PointCloud2 header (width, height, point_step, fields, …) and point_data
is the raw PointCloud2 data buffer.
"""

from __future__ import annotations

import json
import logging
import os
import time

import zmq

BRIDGE_HOST = os.getenv("GO2_BRIDGE_HOST", "localhost")
LIDAR_PORT = int(os.getenv("GO2_ZMQ_LIDAR_PORT", "5558"))

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("go2_lidar")


def main() -> None:
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVTIMEO, 1000)  # short, so Ctrl+C is handled promptly
    sub.connect(f"tcp://{BRIDGE_HOST}:{LIDAR_PORT}")
    sub.subscribe(b"lidar")
    log.info("Subscribed to tcp://%s:%d", BRIDGE_HOST, LIDAR_PORT)

    first = True
    clouds, points = 0, 0
    t_last = t_rx = time.time()

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
            n = meta["width"] * meta["height"]
            if first:
                first = False
                log.info(
                    "First cloud: %d points, %d bytes/point, fields %s, frame '%s'",
                    n, meta["point_step"], [f["name"] for f in meta["fields"]], meta["frame_id"],
                )
            if meta["point_step"] and len(data) != n * meta["point_step"]:
                log.warning("Data size %d doesn't match %d points × %d bytes",
                            len(data), n, meta["point_step"])

            clouds += 1
            points += n
            now = time.time()
            if now - t_last >= 1.0:
                log.info("%.1f clouds/s, %d points/cloud (last: %d)",
                         clouds / (now - t_last), points // clouds, n)
                clouds, points, t_last = 0, 0, now
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        ctx.term()


if __name__ == "__main__":
    main()
