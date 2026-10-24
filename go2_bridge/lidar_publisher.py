"""LiDAR point cloud DDS → ZMQ PUB publisher thread."""

from __future__ import annotations

import json
import logging
import threading

import zmq
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

from . import config

log = logging.getLogger(__name__)


class LidarPublisher:
    """Subscribes to the Go2 LiDAR PointCloud2 topic and republishes it on a ZMQ PUB socket.

    The DDS callback only stores the newest cloud; a separate thread owns the
    ZMQ socket and sends it, so slow subscribers never block DDS.
    """

    def __init__(self, ctx: zmq.Context) -> None:
        self._ctx = ctx
        self._latest: PointCloud2_ | None = None
        self._lock = threading.Lock()
        self._new = threading.Event()
        self._running = False
        self._thread: threading.Thread | None = None
        self._sub: ChannelSubscriber | None = None

    def start(self) -> None:
        if config.LIDAR_SWITCH_ON:
            self._switch_on()

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="lidar-pub")
        self._thread.start()

        self._sub = ChannelSubscriber(config.LIDAR_TOPIC, PointCloud2_)
        self._sub.Init(self._on_cloud)
        log.info("LiDAR publisher started: %s → port %d", config.LIDAR_TOPIC, config.ZMQ_LIDAR_PORT)

    def shutdown(self) -> None:
        self._running = False
        self._new.set()  # wake the publish loop so it can exit
        if self._sub:
            self._sub.Close()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _switch_on(self) -> None:
        pub = ChannelPublisher("rt/utlidar/switch", String_)
        pub.Init()
        msg = std_msgs_msg_dds__String_()
        msg.data = "ON"
        pub.Write(msg)
        log.info("Sent LiDAR switch ON")

    def _on_cloud(self, msg: PointCloud2_) -> None:
        with self._lock:
            self._latest = msg
            self._new.set()

    def _loop(self) -> None:
        pub = self._ctx.socket(zmq.PUB)
        pub.bind(f"tcp://*:{config.ZMQ_LIDAR_PORT}")
        first = True

        while self._running:
            if not self._new.wait(5.0):
                log.warning("No LiDAR data on %s for 5 s", config.LIDAR_TOPIC)
                continue
            if not self._running:
                break
            with self._lock:
                msg = self._latest
                self._new.clear()

            try:
                meta = {
                    "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    "frame_id": msg.header.frame_id,
                    "width": msg.width,
                    "height": msg.height,
                    "point_step": msg.point_step,
                    "row_step": msg.row_step,
                    "is_bigendian": msg.is_bigendian,
                    "is_dense": msg.is_dense,
                    "fields": [
                        {"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count}
                        for f in msg.fields
                    ],
                }
                data = bytes(msg.data)
                pub.send_multipart([b"lidar", json.dumps(meta).encode(), data])

                if first:
                    first = False
                    log.info(
                        "First LiDAR cloud: %d points, fields %s, frame '%s'",
                        msg.width * msg.height,
                        [f.name for f in msg.fields],
                        msg.header.frame_id,
                    )
            except Exception:
                log.exception("LiDAR publish failed")

        pub.close()
