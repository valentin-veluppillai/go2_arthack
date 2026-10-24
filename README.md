# Go2 ZMQ Bridge, CLI Client, Web App & MCP Server

A reusable interface layer for the Unitree Go2 quadruped robot. A ZeroMQ bridge process sits between the Unitree SDK and any number of clients, providing a clean JSON protocol for actions, movement, and camera streaming.

## Architecture

```
┌─────────────┐   ZMQ REQ/REP    ┌──────────────────┐   Unitree SDK2   ┌──────────┐
│  CLI Client  │ ──────────────► │                  │ ───────────────► │          │
└─────────────┘   (port 5555)    │   go2_bridge     │                  │  Go2     │
                                 │                  │ ◄─────────────── │  Robot   │
┌─────────────┐   HTTP/WS        │  ┌─────────────┐ │                  │          │
│  Web App     │ ──────────────► │  │ cmd_handler  │ │                  └──────────┘
│  (FastAPI)   │ ◄── ZMQ SUB ── │  │ move_loop    │ │
└─────────────┘   (port 5556)    │  │ camera_pub   │ │
                                 │  └─────────────┘ │
┌─────────────┐   ZMQ REQ/REP   │                  │
│  MCP Server  │ ──────────────► │                  │
│  (stdio)     │ ◄── ZMQ SUB ── │                  │
└─────────────┘                  └──────────────────┘
```

**go2_bridge** runs five threads:

| Thread | Purpose | Rate |
|--------|---------|------|
| **command_handler** | ZMQ REQ/REP on port 5555 — dispatches action, move, stop, config commands | On demand |
| **movement_loop** | Relays last commanded velocity to the robot via SDK | 20 Hz |
| **camera_publisher** | Grabs JPEG frames from VideoClient, publishes on ZMQ PUB port 5556 | 10 FPS |
| **lidar_publisher** | Subscribes to the LiDAR `PointCloud2` DDS topic, publishes on ZMQ PUB port 5558 | ~15 Hz (sensor rate) |
| **slam_publisher** | Forwards the on-board SLAM (uslam) map-frame scans, pose and log on ZMQ PUB port 5559 | ~5 Hz scans while mapping |

### Movement Safety

The movement loop continuously sends the last commanded velocity at 20 Hz. If no new `move` command arrives within **250 ms**, velocity automatically resets to zero. This gives smooth continuous motion from a joystick or keyboard while guaranteeing a quick stop if the client disconnects or crashes.

### Obstacle Avoidance

Enabled by default. Uses `ObstaclesAvoidClient` for all movement, routing through the robot's built-in obstacle detection. Can be toggled at runtime via the `obstacle_avoidance` command. Falls back to `SportClient.Move()` when disabled.

## Quick Start

### Prerequisites

- Python 3.11+ with the Unitree SDK2 Python package installed
- Network connection to the Go2 robot
- ZeroMQ, FastAPI, and uvicorn

### Install Dependencies

```bash
source go2_py311/bin/activate
pip install -r requirements_bridge.txt
```

### 1. Start the Bridge

```bash
GO2_NETWORK_INTERFACE=eno1 python -m go2_bridge
```

You should see:
```
Go2 Bridge running — CMD port 5555, PUB port 5556
```

### 2. CLI Client

```bash
# Check bridge status
python go2_client/cli_client.py status

# List available actions
python go2_client/cli_client.py list_actions

# Make the robot stand up
python go2_client/cli_client.py action stand_up

# Move forward at 0.3 m/s
python go2_client/cli_client.py move 0.3 0.0 0.0

# Emergency stop
python go2_client/cli_client.py stop

# Save a camera frame
python go2_client/cli_client.py camera_frame

# Toggle obstacle avoidance
python go2_client/cli_client.py obstacle_avoidance 0   # off
python go2_client/cli_client.py obstacle_avoidance 1   # on

# Set speed level (1-3)
python go2_client/cli_client.py speed_level 2
```

### 3. Web App

```bash
python -m go2_webapp
```

Open **http://localhost:8080** in a browser. The UI provides:

- **Live camera feed** via WebSocket
- **WASD + Q/E keyboard controls** for movement and rotation
- **On-screen directional buttons** (touch-friendly)
- **Action buttons** for all registered actions (stand up, sit, dance, flip, etc.)
- **Obstacle avoidance toggle**
- **Speed level selector** (1-3)
- **STOP button** (also spacebar)

### 4. MCP Server (for Claude Code / Claude Desktop)

The MCP server exposes robot control as tools that an LLM can call. It connects to the bridge as a ZMQ client, just like the CLI and web app.

```bash
# Run directly (stdio transport)
python -m go2_mcp

# Test with MCP inspector
mcp dev go2_mcp/server.py
```

**Available MCP tools:**

| Tool | Description |
|------|-------------|
| `get_status` | Get robot status (obstacle avoidance, speed, light) |
| `list_actions` | List all available actions |
| `execute_action` | Run a named action (stand_up, dance1, etc.) |
| `move` | Move with velocity (vx, vy, vyaw) |
| `stop` | Emergency stop |
| `set_obstacle_avoidance` | Toggle obstacle avoidance on/off |
| `set_speed_level` | Set speed 1-3 |
| `set_light` | Turn head light on/off |
| `get_camera_frame` | Capture and return a JPEG frame |

**Claude Desktop configuration** (`claude_desktop_config.json`):
```json
{
  "mcpServers": {
    "go2": {
      "command": "/path/to/go2/go2_py311/bin/python",
      "args": ["/path/to/go2/go2_mcp/server.py"]
    }
  }
}
```

**Claude Code configuration** (`.claude/settings.json` or project settings):
```json
{
  "mcpServers": {
    "go2": {
      "command": "/path/to/go2/go2_py311/bin/python",
      "args": ["-m", "go2_mcp"],
      "cwd": "/path/to/go2"
    }
  }
}
```

### 5. LiDAR Subscriber

The bridge forwards the Go2's LiDAR point cloud (`rt/utlidar/cloud`, `sensor_msgs/PointCloud2`) on port 5558. `go2_lidar` subscribes to it and logs how many points arrive, which is a quick way to check the LiDAR is reachable:

```bash
python -m go2_lidar
# First cloud: 3000 points, 16 bytes/point, fields ['x', 'y', 'z', 'intensity'], frame 'utlidar_lidar'
# 14.6 clouds/s, 3000 points/cloud (last: 3000)
```

It only needs `pyzmq`, so it can run on any machine that can reach the bridge.

### 6. Live 3D Map (uSLAM)

The Go2 runs its own SLAM (`uslam_server`) on board. The bridge forwards its world-frame scans, the robot pose and its log on port 5559, and `go2_map` merges every scan into one map and shows it live in a [Rerun](https://rerun.io) 3D viewer, together with the robot, its path and the current scan. The map keeps everything seen so far, so the whole area around the robot is visible; orbit with the mouse, the view fits the whole map.

```bash
pip install -r requirements_map.txt
python -m go2_map                    # sends mapping/start, opens the viewer
GO2_MAP_WEB=1 python -m go2_map      # viewer in the browser instead
```

uslam only publishes while it is mapping (or localized on a saved map), so `go2_map` sends `mapping/start` when it starts (`GO2_MAP_START=0` to skip). Mapping keeps running after `go2_map` exits. Commands can also be sent by hand:

```bash
python go2_client/cli_client.py slam mapping/start
python go2_client/cli_client.py slam mapping/stop    # saves the map on the robot
```

> **Note:** the robot has a single map slot. `mapping/stop` saves the new map there and **overwrites the previously saved map** (used for localization / navigation in the Unitree app).

The live viewer only keeps the latest map, scan and robot pose (no timeline to scrub back), so its memory stays bounded over long sessions; it is capped at 1 GB (`GO2_MAP_VIEWER_MEMORY`). A `.rrd` recording made with `GO2_MAP_SAVE` keeps the full history.

Indoor ceilings would cover the map, so points more than 1.5 m above the robot's start height are hidden in the viewer (`GO2_MAP_CEILING`).

**Moving objects.** People walking past the robot would leave trails ("ghosts") in the map. `go2_map` keeps a 10 cm occupancy grid next to the map: every scan ray marks the space it passes through as free, so cells that were only briefly occupied are cleared again, while walls and furniture keep getting hit. Scan points that land in space seen free before are drawn in magenta as `world/moving`, which shows people while they are there. Turn the filter off with `GO2_MAP_FILTER=0`.

**Map stream.** `go2_map` also publishes the cleaned map once per second on port 5560 as `[b"occupancy", meta_json, float32 xyz]` (meta: `stamp`, `count`, `voxel`, `z_ref`, `robot` pose) for other apps such as `go2_viz`.

**PLY export.** When you stop `go2_map` with Ctrl+C it writes the whole map (moving objects removed, no ceiling cut) as a binary PLY with height colours, `go2_map_<timestamp>.ply` in the current directory. Set `GO2_MAP_PLY=path.ply` for a fixed name or `GO2_MAP_PLY=0` to skip it.

### 7. Co-Creation Visualization

`go2_viz` is a browser app that renders the room from the occupancy map as particles and keeps rendering continuously while the map and the human/robot inputs change. It implements the human–robodog co-creation spec:

| Input | Range | Visual effect |
|-------|-------|---------------|
| Human–robot **distance** | 0.3 – 5 m | **Jitter** ∝ 1/d², normalised to 1 at ≤ 1 m and 0 at ≥ 3 m: vibration along the surface normals; below 1 m some particles also tear off the surface for a frame |
| Human **movement / velocity** | 0 – 1 | **Density in the view cone**: focus particles ×1 (sparse trickle) to ×15 (burst) |
| Interaction **entropy** | 0 – 1 | **Dispersion** of the focus particles: on the surface at 0, an aerosol cloud up to 0.5 m off the surfaces at 1 |
| Posture **confidence** | 0 – 1 | **Saturation** of the in-cone colouring: monochrome at 0, full colour at 1 |
| Perceived human **scale** | 0 – 1 | Share of particles drawn as **diamond / crosshair** markers instead of circles |
| Head **yaw / pitch**, **cone angle** | °, 15 – 90° | **Zone of interest**: particles inside the cone take on a faked time-of-flight colour (viridis by distance from the head); outside they decay back to gray |

How it works: a calm **base pool** of particles covers the whole room. A second **focus pool** only takes spots inside the view cone; its size is the number of map points in the cone × focus density × (1 + 14 · velocity), so the cone gets denser with movement whether it is narrow or wide. When the cone moves, focus particles whose spot drops out fade out and reappear inside the cone, so nothing flies through empty space. Surface normals for jitter and dispersion are computed on the server (PCA over each point's 12 nearest neighbours).

```bash
pip install -r requirements_viz.txt
python -m go2_viz                                  # live from go2_map (port 5560)
GO2_VIZ_PLY=go2_map_20261024_173512.ply python -m go2_viz   # from a saved map, reloaded when the file changes
```

Open http://localhost:8090. The panel has the test controls from the spec: **Human** (distance, gesture velocity, posture confidence, detected scale, bearing) and **Robot & canvas** (head yaw/pitch, cone angle, head tracks human, entropy). A simulated human (a translucent column) stands at the given distance and bearing; with *head tracks human* the head turns towards it. Without a robot pose (e.g. a PLY file) a virtual robot stands in the middle of the map. The swarm's own behaviour (base particles, focus density, spring, flow drift, hops, size, opacity, colours, view range, desaturation time, marker changes, camera) is in the collapsed **Tuning** folder. The HUD shows the data source, robot source, points and focus particles in the cone, and the current mapping (jitter, dispersion, saturation, marker share). **Snapshot** saves the current frame as PNG, **Record** records the frame stream as WebM.

The server sends at most 80 000 points to the browser (`GO2_VIZ_MAX_POINTS`), thinned evenly so consecutive maps stay similar.

**Driving it from robot data.** `go2_viz` binds a ZMQ SUB socket on port 5561 (`GO2_VIZ_PARAM_PORT`). Any number of processes can connect a PUB socket to it and send `[b"viz", json]` with any subset of the inputs; browsers ease into new numbers over about half a second, so noisy sensor data doesn't flicker:

| Key | Range | Meaning |
|-----|-------|---------|
| `distance` | 0.3 – 5 | Human–robot distance (m) |
| `velocity` | 0 – 1 | Human movement |
| `entropy` | 0 – 1 | Interaction entropy |
| `confidence` | 0 – 1 | Posture confidence |
| `scale` | 0 – 1 | Perceived human scale |
| `headYaw`, `headPitch` | °, ±180 / ±60 | Head direction relative to the body |
| `coneAngle` | 15 – 90 | View cone, full angle (°) |
| `trackHuman` | `true` / `false` | Head follows the human |
| `humanBearing` | ±180 | Direction of the human relative to the body (°) |

```python
pub = zmq.Context().socket(zmq.PUB)
pub.connect("tcp://localhost:5561")
pub.send_multipart([b"viz", json.dumps({"distance": 0.8, "velocity": 0.6}).encode()])
```

For testing from the shell: `python -m go2_viz.send distance=0.6 velocity=0.8 entropy=0.3 confidence=0.9 scale=0.7`. Unknown keys are ignored (and logged); the last values are also sent to browsers that connect later. The HUD shows when inputs last arrived.

## ZMQ Protocol

### Commands (REQ/REP — port 5555)

All messages are JSON. Request format:

```json
{"cmd": "<command>", "params": {<optional params>}}
```

Response format:

```json
{"ok": true, "msg": "description", "data": null}
```

| Command | Params | Description |
|---------|--------|-------------|
| `action` | `{"name": "stand_up"}` | Execute a named action |
| `move` | `{"vx": 0.3, "vy": 0.0, "vyaw": 0.0}` | Set movement velocity |
| `stop` | — | Zero velocity immediately |
| `obstacle_avoidance` | `{"enabled": true}` | Toggle obstacle avoidance |
| `speed_level` | `{"level": 2}` | Set speed (1-3) |
| `light` | `{"on": true}` | Turn head light on/off |
| `slam` | `{"command": "mapping/start"}` | Send a command to the on-board SLAM (uslam) |
| `list_actions` | — | Get list of available actions |
| `status` | — | Get current bridge status |

### Streams (PUB/SUB — port 5556)

| Topic | Payload | Rate |
|-------|---------|------|
| `camera` | Raw JPEG bytes | ~10 FPS |

Subscribe with topic filter `b"camera"`. Frames arrive as two-part multipart messages: `[topic, jpeg_data]`.

### LiDAR Stream (PUB/SUB — port 5558)

| Topic | Payload | Rate |
|-------|---------|------|
| `lidar` | `[topic, meta_json, point_data]` | ~15 Hz |

`meta_json` carries the PointCloud2 header: `stamp`, `frame_id`, `width`, `height`, `point_step`, `row_step`, `is_bigendian`, `is_dense` and `fields` (`name`, `offset`, `datatype`, `count`). `point_data` is the raw PointCloud2 data buffer (`width × height × point_step` bytes), so it can be turned into a NumPy structured array with the field offsets.

### SLAM Stream (PUB/SUB — port 5559)

| Topic | Payload | Source (DDS) |
|-------|---------|--------------|
| `slam_cloud` | `[topic, meta_json, point_data]`, PointCloud2 in the map frame | `rt/uslam/frontend/cloud_world_ds` (mapping), `rt/uslam/localization/cloud_world` (localized) |
| `slam_odom` | JSON `{"stamp", "frame_id", "position": [x, y, z], "orientation": [x, y, z, w]}` | `rt/uslam/frontend/odom`, `rt/uslam/localization/odom` |
| `slam_log` | UTF-8 text, e.g. `mapping/start/success` | `rt/uslam/server_log` |

The `slam` command (`{"cmd": "slam", "params": {"command": "mapping/start"}}`) publishes any uslam command string on `rt/uslam/client_command`; see [unitree_ui's SLAM notes](https://github.com/legion1581/unitree_ui/blob/main/docs/slam.md) for the command set.

## Available Actions

| Action | Robot Behavior |
|--------|---------------|
| `stand_up` | Stand up from sitting/lying |
| `stand_down` | Lie down |
| `balance_stand` | Enter balance stand mode |
| `recovery_stand` | Recovery stand (from fallen) |
| `sit` | Sit down |
| `hello` | Wave hello |
| `stretch` | Stretching motion |
| `dance1` | Dance routine 1 |
| `dance2` | Dance routine 2 |
| `heart` | Heart gesture |
| `front_flip` | Front flip |
| `front_jump` | Front jump |
| `back_flip` | Back flip |
| `left_flip` | Left flip |
| `hand_stand` | Handstand |
| `damp` | Emergency soft stop (all motors) |
| `stop_move` | Stop current movement |

## Configuration

All settings via environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `GO2_NETWORK_INTERFACE` | `eno1` | DDS network interface for SDK |
| `GO2_ZMQ_CMD_PORT` | `5555` | REQ/REP command port |
| `GO2_ZMQ_PUB_PORT` | `5556` | PUB/SUB stream port |
| `GO2_MOVE_HZ` | `20` | Movement loop rate |
| `GO2_MOVE_TIMEOUT_MS` | `250` | Movement command timeout |
| `GO2_CAMERA_FPS` | `10` | Camera publish rate |
| `GO2_OBSTACLE_AVOIDANCE` | `1` | Enable obstacle avoidance at startup |
| `GO2_BRIDGE_HOST` | `localhost` | Bridge host (for clients) |
| `GO2_WEBAPP_HOST` | `0.0.0.0` | Web app bind address |
| `GO2_WEBAPP_PORT` | `8080` | Web app port |
| `GO2_LIDAR` | `1` | Enable the LiDAR publisher in the bridge |
| `GO2_LIDAR_TOPIC` | `rt/utlidar/cloud` | DDS PointCloud2 topic to forward |
| `GO2_LIDAR_SWITCH_ON` | `1` | Send `ON` to `rt/utlidar/switch` at startup |
| `GO2_ZMQ_LIDAR_PORT` | `5558` | LiDAR PUB/SUB port |
| `GO2_SLAM` | `1` | Forward the on-board SLAM (uslam) streams in the bridge |
| `GO2_ZMQ_SLAM_PORT` | `5559` | SLAM PUB/SUB port |
| `GO2_MAP_VOXEL` | `0.05` | Map resolution in metres (one point per voxel) |
| `GO2_MAP_CEILING` | `1.5` | Hide points this far above the robot's start height (0 = show all) |
| `GO2_MAP_START` | `1` | `go2_map` sends `mapping/start` at startup |
| `GO2_MAP_WEB` | `0` | Open the viewer in the browser instead of the native Rerun window |
| `GO2_MAP_SAVE` | — | Record to this `.rrd` file instead of showing a viewer |
| `GO2_MAP_VIEWER_MEMORY` | `1GB` | Memory cap of the live Rerun viewer |
| `GO2_MAP_FILTER` | `1` | Remove moving objects (people walking past) from the map |
| `GO2_MAP_FILTER_CELL` | `0.1` | Cell size in metres of the occupancy grid used by the filter |
| `GO2_MAP_PLY` | `go2_map_<timestamp>.ply` | Map file written on exit (`0` = off) |
| `GO2_MAP_PUB_PORT` | `5560` | Port on which `go2_map` publishes the cleaned map |
| `GO2_MAP_HOST` | `GO2_BRIDGE_HOST` | Host running `go2_map` (for `go2_viz`) |
| `GO2_VIZ_PLY` | — | Show this PLY file instead of the live map |
| `GO2_VIZ_PORT` | `8090` | `go2_viz` web server port |
| `GO2_VIZ_MAX_POINTS` | `80000` | Most map points sent to the browser |
| `GO2_VIZ_PARAM_PORT` | `5561` | Port where `go2_viz` receives human/robot inputs from robot-data publishers |

## Writing Custom Clients

The bridge speaks plain JSON over ZeroMQ, so you can write clients in any language with a ZMQ binding:

```python
import zmq

ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.connect("tcp://localhost:5555")

# Send a command
sock.send_json({"cmd": "action", "params": {"name": "hello"}})
response = sock.recv_json()
print(response)  # {"ok": true, "msg": "hello executed", "data": {"code": 0}}

# Subscribe to camera
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://localhost:5556")
sub.subscribe(b"camera")
topic, jpeg_data = sub.recv_multipart()
with open("frame.jpg", "wb") as f:
    f.write(jpeg_data)
```

## Shutdown

Press `Ctrl+C` to stop the bridge. It will:
1. Zero all velocity commands
2. Call `UseRemoteCommandFromApi(False)` to restore normal robot control
3. Clean up ZMQ sockets
