"""uSLAM (on-board SLAM) DDS → ZMQ PUB publisher thread."""

from __future__ import annotations

import json
import logging
import queue
import threading

import zmq
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

from . import config
from .protocol import cloud_meta

log = logging.getLogger(__name__)

# uslam_server publishes the frontend topics while mapping and the
# localization topics while localized on a saved map; both are forwarded on
# the same ZMQ topics since only one pair is active at a time.
CLOUD_TOPICS = ("rt/uslam/frontend/cloud_world_ds", "rt/uslam/localization/cloud_world")
ODOM_TOPICS = ("rt/uslam/frontend/odom", "rt/uslam/localization/odom")
LOG_TOPIC = "rt/uslam/server_log"


class SlamPublisher:
    """Forwards uSLAM world-frame clouds, odometry and server log lines on a ZMQ PUB socket.

        [b"slam_cloud", meta_json, point_data]   PointCloud2 in the map frame
        [b"slam_odom",  json]                    {"stamp", "frame_id", "position", "orientation"}
        [b"slam_log",   utf-8 text]              command acks and state changes

    DDS callbacks only encode and enqueue; a separate thread owns the socket.
    """

    def __init__(self, ctx: zmq.Context) -> None:
        self._ctx = ctx
        self._queue: queue.Queue[list[bytes]] = queue.Queue(maxsize=100)
        self._running = False
        self._thread: threading.Thread | None = None
        self._subs: list[ChannelSubscriber] = []
        self._first_cloud = True

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="slam-pub")
        self._thread.start()

        for topic in CLOUD_TOPICS:
            self._subscribe(topic, PointCloud2_, self._on_cloud)
        for topic in ODOM_TOPICS:
            self._subscribe(topic, Odometry_, self._on_odom)
        self._subscribe(LOG_TOPIC, String_, self._on_log)
        log.info("SLAM publisher started on port %d (send 'mapping/start' to begin mapping)",
                 config.ZMQ_SLAM_PORT)

    def shutdown(self) -> None:
        self._running = False
        for sub in self._subs:
            sub.Close()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _subscribe(self, topic: str, msg_type, handler) -> None:
        sub = ChannelSubscriber(topic, msg_type)
        sub.Init(handler)
        self._subs.append(sub)

    def _put(self, frames: list[bytes]) -> None:
        try:
            self._queue.put_nowait(frames)
        except queue.Full:
            pass  # nobody is keeping up; dropping is better than blocking DDS

    def _on_cloud(self, msg: PointCloud2_) -> None:
        if self._first_cloud:
            self._first_cloud = False
            log.info("First SLAM cloud: %d points in frame '%s'", msg.width * msg.height, msg.header.frame_id)
        self._put([b"slam_cloud", json.dumps(cloud_meta(msg)).encode(), bytes(msg.data)])

    def _on_odom(self, msg: Odometry_) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._put([b"slam_odom", json.dumps({
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame_id": msg.header.frame_id,
            "position": [p.x, p.y, p.z],
            "orientation": [q.x, q.y, q.z, q.w],
        }).encode()])

    def _on_log(self, msg: String_) -> None:
        log.info("uslam: %s", msg.data)
        self._put([b"slam_log", msg.data.encode()])

    def _loop(self) -> None:
        pub = self._ctx.socket(zmq.PUB)
        pub.bind(f"tcp://*:{config.ZMQ_SLAM_PORT}")
        while self._running:
            try:
                frames = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            pub.send_multipart(frames)
        pub.close()
