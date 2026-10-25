"""Particle-swarm map visualization: python -m go2_viz

Serves a browser app (http://localhost:8090) that renders the occupancy map
from go2_map as a swarm of particles. The map comes either live from
go2_map's PUB socket (port 5560) or from a PLY file (GO2_VIZ_PLY), which is
reloaded whenever it changes. Every new map is pushed to all browsers over a
WebSocket; the swarm then flows to the new geometry.

The visualization is driven by human/robot inputs: any process can connect a
ZMQ PUB socket to tcp://<this host>:5561 and send [b"viz", json] with any of
    distance      human–robot distance, m (0.3–5)           → jitter
    velocity      human movement, 0–1                       → density in the view cone
    entropy       interaction entropy, 0–1                  → dispersion in the view cone
    confidence    posture confidence, 0–1                   → colour saturation
    scale         perceived human scale, 0–1                → share of diamond/crosshair markers
    headYaw       head yaw relative to the body, degrees
    headPitch     head pitch, degrees
    coneAngle     view cone angle, degrees (15–90)
    trackHuman    true = the head follows the human
    humanBearing  direction of the human relative to the body, degrees
    boxLeft, boxRight, boxTop, boxBottom
                  the human's bounding box as angles from the camera axis, degrees (+ = left / up);
                  shapes the vignette of the human colour channel
The browser maps these to the visual parameters; the latest values are
forwarded to every browser, which eases into them.

Human channel colours: go2_map colours its points from the front camera and
sends them as RGBA (alpha = how well a colour is known). For offline tests,
GO2_VIZ_TEXTURE ("pattern" for a built-in test panorama, or an image file)
colours maps that come without colours instead, wrapped around the middle of
the map and blurred by GO2_VIZ_BLUR (a fraction of the image width). The
browser screen-blends the colour with the robot's time-of-flight colour
around the human inside the view cone.

WebSocket messages: the map is binary (uint32 header length, JSON header,
padding to a multiple of 4 bytes, then float32 x, y, z triples followed by
float32 surface normals, then uint8 r, g, b, a per point if the header has
"rgba": true); input updates are text, {"params": {...}}.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import struct
from contextlib import asynccontextmanager
from pathlib import Path

import numpy as np
import uvicorn
import zmq
import zmq.asyncio
from scipy.spatial import cKDTree
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from . import texture

MAP_HOST = os.getenv("GO2_MAP_HOST", os.getenv("GO2_BRIDGE_HOST", "localhost"))
MAP_PUB_PORT = int(os.getenv("GO2_MAP_PUB_PORT", "5560"))
VIZ_PLY = os.getenv("GO2_VIZ_PLY", "")  # use this PLY file instead of the live map
VIZ_HOST = os.getenv("GO2_VIZ_HOST", "0.0.0.0")
VIZ_PORT = int(os.getenv("GO2_VIZ_PORT", "8090"))
VIZ_MAX_POINTS = int(os.getenv("GO2_VIZ_MAX_POINTS", "80000"))  # sent to the browser
VIZ_PARAM_PORT = int(os.getenv("GO2_VIZ_PARAM_PORT", "5561"))  # SUB binds here; publishers connect
VIZ_TEXTURE = os.getenv("GO2_VIZ_TEXTURE", "")  # human channel: "pattern" or an image file (panorama)
VIZ_BLUR = float(os.getenv("GO2_VIZ_BLUR", "0.03"))  # blur sigma, fraction of the texture width
REMOTE_PARAMS = {"distance", "velocity", "entropy", "confidence", "scale",
                 "headYaw", "headPitch", "coneAngle", "trackHuman", "humanBearing",
                 "boxLeft", "boxRight", "boxTop", "boxBottom"}
NORMAL_NEIGHBOURS = 12

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("go2_viz")

STATIC_DIR = Path(__file__).parent / "static"
clients: set[WebSocket] = set()
latest: bytes | None = None  # last encoded map, sent to new clients
remote_params: dict = {}  # merged parameter updates, sent to new clients
panorama: np.ndarray | None = None  # blurred human-channel texture, loaded at startup

_PLY_TYPES = {"char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1", "short": "i2", "int16": "i2",
              "ushort": "u2", "uint16": "u2", "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
              "float": "f4", "float32": "f4", "double": "f8", "float64": "f8"}


def load_ply(path: str) -> np.ndarray:
    """x, y, z of the vertex element of an ASCII or binary PLY (other properties are skipped)."""
    raw = Path(path).read_bytes()
    end = raw.index(b"end_header") + len(b"end_header")
    end = raw.index(b"\n", end) + 1
    header = raw[:end].decode(errors="replace").splitlines()
    fmt = next(line.split()[1] for line in header if line.startswith("format"))
    count, props, in_vertex = 0, [], False
    for line in header:
        parts = line.split()
        if parts[:1] == ["element"]:
            in_vertex = parts[1] == "vertex"
            if in_vertex:
                count = int(parts[2])
        elif parts[:1] == ["property"] and in_vertex:
            props.append((parts[-1], _PLY_TYPES[parts[1]]))
    if fmt == "ascii":
        names = [name for name, _ in props]
        rows = np.loadtxt(raw[end:].decode().splitlines()[:count], ndmin=2)
        return rows[:, [names.index(a) for a in "xyz"]].astype(np.float32)
    order = "<" if fmt == "binary_little_endian" else ">"
    rec = np.frombuffer(raw[end:], dtype=np.dtype([(n, order + t) for n, t in props]), count=count)
    return np.stack([rec["x"], rec["y"], rec["z"]], axis=1).astype(np.float32)


def estimate_normals(points: np.ndarray) -> np.ndarray:
    """Unit surface normal per point: direction of least spread among its nearest neighbours (PCA)."""
    if len(points) <= NORMAL_NEIGHBOURS:
        return np.tile(np.array([0, 0, 1], dtype=np.float32), (len(points), 1))
    _, idx = cKDTree(points).query(points, k=NORMAL_NEIGHBOURS, workers=-1)
    nb = points[idx] - points[idx].mean(axis=1, keepdims=True)
    _, vecs = np.linalg.eigh(np.einsum("nki,nkj->nij", nb, nb))  # eigenvalues ascending
    return vecs[:, :, 0].astype(np.float32)


def encode(points: np.ndarray, meta: dict, rgba: np.ndarray | None = None) -> bytes:
    """Thin the cloud to VIZ_MAX_POINTS (evenly, so consecutive maps stay similar), add normals and
    colours (the map's own, else the test texture's), pack it."""
    if len(points) > VIZ_MAX_POINTS:
        keep = np.linspace(0, len(points) - 1, VIZ_MAX_POINTS).astype(int)
        points = points[keep]
        rgba = rgba[keep] if rgba is not None else None
    points = points.astype(np.float64)
    normals = estimate_normals(points)
    if rgba is None and panorama is not None and len(points):
        rgb = texture.panorama_colors(points, panorama, texture.panorama_centre(points))
        rgba = np.concatenate([rgb, np.full((len(points), 1), 255, dtype=np.uint8)], axis=1)
    header = json.dumps({**meta, "count": len(points), "rgba": rgba is not None}).encode()
    pad = (-(4 + len(header))) % 4
    return (struct.pack("<I", len(header)) + header + b" " * pad + points.astype("<f4").tobytes()
            + normals.astype("<f4").tobytes() + (rgba.astype(np.uint8).tobytes() if rgba is not None else b""))


async def broadcast(message: bytes) -> None:
    global latest
    latest = message
    for ws in list(clients):
        try:
            await ws.send_bytes(message)
        except Exception:
            clients.discard(ws)


async def broadcast_params(update: dict) -> None:
    remote_params.update(update)
    message = json.dumps({"params": update})
    for ws in list(clients):
        try:
            await ws.send_text(message)
        except Exception:
            clients.discard(ws)


async def param_source() -> None:
    """Parameter updates from robot-data publishers; the SUB socket binds so any number can connect."""
    ctx = zmq.asyncio.Context()
    sub = ctx.socket(zmq.SUB)
    sub.bind(f"tcp://*:{VIZ_PARAM_PORT}")
    sub.subscribe(b"viz")
    log.info("Listening for parameter updates on port %d", VIZ_PARAM_PORT)
    try:
        while True:
            _topic, raw = await sub.recv_multipart()
            try:
                update = json.loads(raw)
            except ValueError:
                log.warning("Ignoring non-JSON parameter message: %r", raw[:80])
                continue
            unknown = set(update) - REMOTE_PARAMS
            if unknown:
                log.warning("Ignoring unknown parameters %s", sorted(unknown))
            update = {k: v for k, v in update.items() if k in REMOTE_PARAMS}
            if update:
                await broadcast_params(update)
    finally:
        sub.close()
        ctx.term()


async def zmq_source() -> None:
    ctx = zmq.asyncio.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 2)  # only the newest maps matter
    sub.connect(f"tcp://{MAP_HOST}:{MAP_PUB_PORT}")
    sub.subscribe(b"occupancy")
    log.info("Listening for go2_map on tcp://%s:%d", MAP_HOST, MAP_PUB_PORT)
    try:
        while True:
            _topic, meta_raw, data, *colors = await sub.recv_multipart()
            meta = json.loads(meta_raw)
            points = np.frombuffer(data, dtype="<f4").reshape(-1, 3)
            rgba = np.frombuffer(colors[0], dtype=np.uint8).reshape(-1, 4) if colors else None  # older go2_map: none
            await broadcast(await asyncio.to_thread(encode, points, {**meta, "source": "go2_map"}, rgba))
    finally:
        sub.close()
        ctx.term()


async def ply_source(path: str) -> None:
    mtime = None
    log.info("Watching %s", path)
    while True:
        try:
            current = os.path.getmtime(path)
            if current != mtime:
                points = load_ply(path)
                mtime = current
                log.info("Loaded %d points from %s", len(points), path)
                await broadcast(await asyncio.to_thread(encode, points, {
                    "source": os.path.basename(path), "stamp": current, "z_ref": None, "robot": None}))
        except Exception:
            log.exception("Could not read %s", path)
        await asyncio.sleep(1.0)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    global panorama
    if VIZ_TEXTURE:
        image = texture.test_pattern() if VIZ_TEXTURE == "pattern" else texture.load_image(VIZ_TEXTURE)
        panorama = texture.blur(image, VIZ_BLUR, wrap=True)
        log.info("Human channel: %s, blurred by %.0f%% of its width", VIZ_TEXTURE, VIZ_BLUR * 100)
    source = asyncio.create_task(ply_source(VIZ_PLY) if VIZ_PLY else zmq_source())
    params = asyncio.create_task(param_source())
    yield
    source.cancel()
    params.cancel()


app = FastAPI(title="Go2 Map Viz", lifespan=lifespan)


@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "index.html")


@app.websocket("/ws/map")
async def ws_map(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    if latest is not None:
        await websocket.send_bytes(latest)
    if remote_params:
        await websocket.send_text(json.dumps({"params": remote_params}))
    try:
        while True:
            await websocket.receive_text()  # keep-alive; the client sends nothing meaningful
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(websocket)


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

if __name__ == "__main__":
    log.info("Open http://localhost:%d", VIZ_PORT)
    uvicorn.run(app, host=VIZ_HOST, port=VIZ_PORT, log_level="warning")
