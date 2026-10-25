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

**Camera colours.** `go2_map` also colours the map from the front camera (bridge port 5556). Up to 3 times per second (`GO2_MAP_COLOR_HZ`) the newest frame is shrunk to 160 px wide and blurred heavily (`GO2_MAP_COLOR_BLUR`, sigma 3 % of the width), so pose timing and calibration errors only shift soft gradients. The map points within 5 m (`GO2_MAP_COLOR_RANGE`) are projected into it with the SLAM pose, through the camera's position from the URDF, its field of view (`GO2_CAMERA_HFOV`, 120° until calibrated) and downward tilt (`GO2_CAMERA_PITCH`, 0°); lens distortion is ignored. A coarse depth buffer that also holds the latest scan keeps surfaces behind others, or behind a person standing in front of them, from taking their colour. Each point keeps a running average, so colours settle over a few frames and follow slow lighting changes. Frames are skipped while the robot turns faster than 0.6 rad/s or the pose is older than 0.3 s. The viewer shows camera colours where a point has been seen and height colours elsewhere. Turn it off with `GO2_MAP_COLOR=0`.

**Map stream.** `go2_map` also publishes the cleaned map once per second on port 5560 as `[b"occupancy", meta_json, float32 xyz, uint8 rgba]` (meta: `stamp`, `count`, `voxel`, `z_ref`, `robot` pose; rgba: camera colour per point, alpha = how well it is known, 0 = never seen, 255 after 3 frames) for other apps such as `go2_viz`.

**PLY export.** When you stop `go2_map` with Ctrl+C it writes the whole map (moving objects removed, no ceiling cut) as a binary PLY with camera colours (height colours where the camera never looked), `go2_map_<timestamp>.ply` in the current directory. Set `GO2_MAP_PLY=path.ply` for a fixed name or `GO2_MAP_PLY=0` to skip it.

### 7. YOLO Object Detection

`go2_yolo` subscribes to the camera stream, runs [Ultralytics YOLO](https://docs.ultralytics.com) on the newest frame, shows the annotated feed in a window and republishes the results on its own PUB socket (port 5557). It doesn't need the Unitree SDK, so it can run in any Python 3.9+ environment on a machine that can reach the bridge.

```bash
pip install -r requirements_yolo.txt
python -m go2_yolo                          # window with annotated feed (q / Esc to quit)
GO2_YOLO_SHOW=0 python -m go2_yolo          # headless, only publishes
GO2_YOLO_DEVICE=mps python -m go2_yolo      # Apple GPU (use 0 for CUDA)
```

The model weights (`yolo26n-pose.pt` by default, a pose model) are downloaded on first run. Frames that arrive while a frame is being processed are dropped, so detection always runs on the latest image.

| Topic (port 5557) | Payload |
|-------|---------|
| `camera_yolo` | Annotated JPEG bytes |
| `detections` | JSON: `{"ts", "width", "height", "detections": [{"cls", "conf", "box": [x1, y1, x2, y2], "keypoints"}]}` |

With a pose model every detection also carries `keypoints`: the 17 COCO body joints as `[x, y, conf]` in pixels (nose, eyes, ears, shoulders, elbows, wrists, hips, knees, ankles). Detection-only models leave the field out.

### 8. Person Follow

`go2_follow` keeps a person in the camera frame using the YOLO detections. It picks the largest person and applies two simple rules:

- **Turn:** box centre in the left / right 40 % of the image → turn left / right
- **Distance:** box smaller than 5 % of the frame → walk forward; larger than 20 % → walk backward

**Deadman switch:** the robot only moves while **SPACE is held** in the follower window. Releasing it, or the window losing focus, sends an immediate `stop`. If the follower crashes, the bridge's 250 ms movement timeout stops the robot.

```bash
pip install -r requirements_follow.txt
python go2_client/cli_client.py action stand_up    # terminal 1, with the bridge running
GO2_YOLO_SHOW=0 python -m go2_yolo                 # terminal 2
python -m go2_follow                               # terminal 3: hold SPACE to follow, q / Esc to quit
```

The window shows the annotated camera feed, the turn zones (yellow lines) and the current command. Obstacle avoidance stays active if it is enabled in the bridge.

### 9. Co-Creation Visualization

`go2_viz` is a browser app that renders the room from the occupancy map as particles and keeps rendering continuously while the map and the human/robot inputs change. It implements the human–robodog co-creation spec:

| Input | Range | Visual effect |
|-------|-------|---------------|
| Human–robot **distance** | 0.3 – 5 m | **Jitter** ∝ 1/d², normalised to 1 at ≤ 1 m and 0 at ≥ 3 m: vibration along the surface normals; below 1 m some particles also tear off the surface for a frame |
| Human **movement / velocity** | 0 – 1 | **Density in the view cone**: focus particles ×1 (sparse trickle) to ×15 (burst) |
| Interaction **entropy** | 0 – 1 | **Dispersion** of the focus particles: on the surface at 0, an aerosol cloud up to 0.5 m off the surfaces at 1 |
| Posture **confidence** | 0 – 1 | **Saturation** of the in-cone colouring: monochrome at 0, full colour at 1 |
| Perceived human **scale** | 0 – 1 | Share of particles drawn as **diamond / crosshair** markers instead of circles; each marker is inflated by a random amount (1× up to *max inflation*, 3× by default, mostly modest) |
| Head **yaw / pitch**, **cone angle** | °, 15 – 90° | **Zone of interest**: particles inside the cone take on a faked time-of-flight colour (viridis by distance from the head); outside they decay back to gray |
| **Proximity** and **gesture velocity** (engagement), human **bounding box** | 0 – 1, ° | **Dual-stream colour** around the human: see below |

How it works: a calm **base pool** of particles covers the whole room. A second **focus pool** only takes spots inside the view cone; its size is the number of map points in the cone × focus density × (1 + 14 · velocity), so the cone gets denser with movement whether it is narrow or wide. When the cone moves, focus particles whose spot drops out fade out and reappear inside the cone, so nothing flies through empty space. Surface normals for jitter and dispersion are computed on the server (PCA over each point's 12 nearest neighbours).

```bash
pip install -r requirements_viz.txt
python -m go2_viz                                  # live from go2_map (port 5560)
GO2_VIZ_PLY=go2_map_20261024_173512.ply python -m go2_viz   # from a saved map, reloaded when the file changes
```

Open http://localhost:8090. The panel has the test controls from the spec: **Human** (distance, gesture velocity, posture confidence, detected scale, bearing) and **Robot & canvas** (head yaw/pitch, cone angle, head tracks human, entropy). A simulated human (a translucent column) stands at the given distance and bearing; with *head tracks human* the head turns towards it. Without a robot pose (e.g. a PLY file) a virtual robot stands in the middle of the map. The swarm's own behaviour (base particles, focus density, spring, flow drift, hops, size, opacity, colours, view range, desaturation time, marker changes, camera) is in the collapsed **Tuning** folder. The HUD shows the data source, robot source, points and focus particles in the cone, and the current mapping (jitter, dispersion, saturation, marker share). **Snapshot** saves the current frame as PNG, **Record** records the frame stream as WebM.

**Tuning and demos.**
- **Presets** (panel → Presets) save every panel setting plus the camera view: *save to file* / *load from file* (JSON), *save in browser* (restored on the next load in that browser), *save to server* (writes `GO2_VIZ_PRESET`, which every browser loads at startup unless it has its own browser preset), *reset to defaults*.
- **Render** switches each layer on or off: base particles, focus particles, robot, view cone, human column, HUD. *Particles only* turns off everything but the particles.
- **Hotkeys:** `h` hides all UI (panel, HUD, overlays, mouse cursor) for a clean view, `f` toggles fullscreen.
- [go2_viz/TUNING.md](go2_viz/TUNING.md) lists every variable that shapes the look, with file and line: panel settings, the input → visual formulas, renderer constants and the data-side environment variables.

```bash
GO2_VIZ_PRESET=demo_look.json python -m go2_viz   # tune, "save to server", and every browser starts with that look
```

**Dual-stream colour.** Inside the cone, where the human is, two colour channels are screen-blended: the robot channel (the time-of-flight false colour) and the human channel (photographic RGB of the map point):

    C_final = 1 − (1 − w_r · C_robot) × (1 − w_h · C_human),   w_h = engagement × vignette,   w_r = 1 − w_h

- **Engagement** = 1 − (1 − proximity)(1 − velocity), where proximity is the jitter term (1 at ≤ 1 m, 0 at ≥ 3 m). Either closeness or movement engages, so the human channel dominates up close; far and still it is pure robot colour. The weights sum to 1, so the blend never washes out to white.
- **Vignette**: the human channel only shows where the human is. Its shape is the human's bounding box as angles seen from the front camera (`boxLeft`/`boxRight`/`boxTop`/`boxBottom` from `go2_viz.feed`), a rounded rectangle that fades out from 0.7 to 1.3 box half-sizes, limited in depth to ±0.4–0.9 m around the human's distance so the wall behind them stays robot-coloured. Without box inputs for 2 s the simulated human column's box is used. The HUD shows the engagement and which box is in use.
- **Human channel source**: live, the camera colours `go2_map` gives each map point (see *Camera colours* above), weighted per point by how well each colour is known, so points the camera never saw stay pure robot colour. For offline tests with a map that has no colours, `GO2_VIZ_TEXTURE=pattern` wraps a built-in test panorama around the middle of the map (12 hue sectors around the horizon, checkered in brightness every 15° of elevation); `GO2_VIZ_TEXTURE=room.jpg` uses an equirectangular image instead (needs Pillow). The texture is blurred heavily (`GO2_VIZ_BLUR`, sigma as a fraction of its width, default 3 %) so small misalignments only shift soft gradients, like the camera frames in `go2_map`. **Tuning → Look → human channel** turns the blend off.

```bash
GO2_VIZ_PLY=go2_map_20261024_175506.ply GO2_VIZ_TEXTURE=pattern python -m go2_viz
```

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
| `boxLeft`, `boxRight`, `boxTop`, `boxBottom` | ±90 | The human's bounding box as angles from the front camera's axis (°, + = left / up); shapes the human-channel vignette |

```python
pub = zmq.Context().socket(zmq.PUB)
pub.connect("tcp://localhost:5561")
pub.send_multipart([b"viz", json.dumps({"distance": 0.8, "velocity": 0.6}).encode()])
```

For testing from the shell: `python -m go2_viz.send distance=0.6 velocity=0.8 entropy=0.3 confidence=0.9 scale=0.7`. Unknown keys are ignored (and logged); the last values are also sent to browsers that connect later. The HUD shows when inputs last arrived.

**Live inputs from the camera and LiDAR.** `python -m go2_viz.feed` turns `go2_yolo`'s pose detections and the bridge's SLAM scans into these inputs and publishes them about 10 times per second. It follows the largest confident person:

| Input | Computed from |
|-------|---------------|
| `humanBearing` | Offset of the box centre from the image centre, through the camera's horizontal field of view (`GO2_CAMERA_HFOV`) |
| `distance` | **LiDAR:** the nearest cluster of scan points in the person's direction (middle half of the box), at least 0.5 m tall, measured horizontally from the body centre. **Fallback** when there are no scans or no such cluster: pinhole estimate from the torso length (shoulders → hips, `GO2_FEED_TORSO`) and the box height (`GO2_FEED_PERSON_H`), the nearer of the two |
| `scale` | Share of the frame the box covers, 1 at `GO2_FEED_SCALE_FULL` |
| `confidence` | Mean keypoint confidence |
| `velocity` | Mean keypoint speed in box heights per second, minus keypoint jitter, 1 at `GO2_FEED_VEL_FULL` |
| `entropy` | Normalised Shannon entropy of the velocity over the last `GO2_FEED_ENTROPY_S` seconds: 0 when movement is steady, 1 when erratic |
| `trackHuman` | `true` while a person is seen; `headYaw` / `headPitch` stay 0, as the Go2 has no separate head |
| `boxLeft` … `boxBottom` | The box edges through the camera's field of view, assuming a level camera |

Without a person for `GO2_FEED_LOST_S` the inputs go back to neutral (5 m, no movement, no confidence, scale 0, head straight ahead). The robot turning moves the whole image, so it counts as human movement.

The LiDAR distance uses the SLAM stream (port 5559): uslam's world-frame scans from the last 0.5 s (`GO2_FEED_LIDAR_WINDOW_S`, the L1 is sparse) are moved into the body frame with uslam's own pose, so the frames always match. uslam only publishes while mapping, which `go2_map` starts. Points between 0.15 m above the floor and head height are searched, and clusters count distinct 5 cm voxels, so a single stray return repeated in every scan doesn't win. An object between the robot and the person (a chair, a table) is measured instead of the person. The log line shows once per second whether the distance came from `lidar` or `camera`; the camera estimate depends on `GO2_CAMERA_HFOV` and the body sizes, so calibrate those for the fallback.

```bash
python -m go2_bridge          # terminal 1 (with GO2_NETWORK_INTERFACE set)
python -m go2_map             # terminal 2: live map
GO2_YOLO_SHOW=0 python -m go2_yolo   # terminal 3: pose detection
python -m go2_viz             # terminal 4: http://localhost:8090
python -m go2_viz.feed        # terminal 5: detections → viz inputs
```

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
| `GO2_YOLO_PUB_PORT` | `5557` | YOLO results PUB port |
| `GO2_YOLO_MODEL` | `yolo26n-pose.pt` | YOLO weights (name or path) |
| `GO2_YOLO_CONF` | `0.4` | Detection confidence threshold |
| `GO2_YOLO_DEVICE` | auto | Inference device (`cpu`, `mps`, `0`, …) |
| `GO2_YOLO_SHOW` | `1` | Show the annotated feed in a window |
| `GO2_YOLO_HOST` | `GO2_BRIDGE_HOST` | Host running `go2_yolo` (for `go2_follow`) |
| `GO2_FOLLOW_EDGE` | `0.4` | Turn when the person's centre is within this fraction of the left/right edge |
| `GO2_FOLLOW_AREA_FAR` | `0.05` | Walk forward when the person box is smaller than this fraction of the frame |
| `GO2_FOLLOW_AREA_NEAR` | `0.20` | Walk backward when the person box is larger than this fraction of the frame |
| `GO2_FOLLOW_VX` | `0.3` | Forward/backward speed (m/s) |
| `GO2_FOLLOW_VYAW` | `0.7` | Turn speed (rad/s) |
| `GO2_FOLLOW_CONF` | `0.5` | Minimum confidence for a person detection |
| `GO2_MAP_PUB_PORT` | `5560` | Port on which `go2_map` publishes the cleaned map |
| `GO2_MAP_COLOR` | `1` | Colour the map from the front camera |
| `GO2_MAP_COLOR_HZ` | `3` | Camera frames used per second for colouring |
| `GO2_MAP_COLOR_BLUR` | `0.03` | Blur of the camera frame before sampling (sigma, fraction of its width) |
| `GO2_MAP_COLOR_RANGE` | `5` | Farthest map points coloured from the camera (m) |
| `GO2_MAP_HOST` | `GO2_BRIDGE_HOST` | Host running `go2_map` (for `go2_viz`) |
| `GO2_VIZ_PLY` | — | Show this PLY file instead of the live map |
| `GO2_VIZ_PORT` | `8090` | `go2_viz` web server port |
| `GO2_VIZ_MAX_POINTS` | `80000` | Most map points sent to the browser |
| `GO2_VIZ_PARAM_PORT` | `5561` | Port where `go2_viz` receives human/robot inputs from robot-data publishers |
| `GO2_VIZ_PARAM_HOST` | `localhost` | Host running `go2_viz` (for `go2_viz.feed` and `go2_viz.send`) |
| `GO2_VIZ_TEXTURE` | — | Human colour channel for offline tests: `pattern` or an equirectangular image file |
| `GO2_VIZ_BLUR` | `0.03` | Blur of the human channel texture (sigma, fraction of its width) |
| `GO2_VIZ_PRESET` | — | Preset file (panel settings + view) browsers load at startup; written by *save to server* |
| `GO2_CAMERA_HFOV` | `120` | Horizontal field of view of the front camera (°): `go2_viz.feed` bearing/distance and `go2_map` colours |
| `GO2_CAMERA_PITCH` | `0` | How far the front camera looks down (°), for `go2_viz.feed` box angles and `go2_map` colours |
| `GO2_FEED_PERSON_H` | `1.7` | Assumed body height (m) for the distance estimate |
| `GO2_FEED_TORSO` | `0.5` | Assumed shoulder-to-hip length (m) for the distance estimate |
| `GO2_FEED_CONF` | `0.5` | Minimum confidence for a person detection |
| `GO2_FEED_SCALE_FULL` | `0.25` | Box share of the frame that gives scale 1 |
| `GO2_FEED_VEL_FULL` | `0.4` | Keypoint speed (box heights/s) that gives velocity 1 |
| `GO2_FEED_ENTROPY_S` | `3` | Window for the entropy (s) |
| `GO2_FEED_LOST_S` | `1.0` | No person for this long → neutral inputs |
| `GO2_FEED_LIDAR` | `1` | Measure the distance from the SLAM scans (`0` = camera estimate only) |
| `GO2_FEED_LIDAR_WINDOW_S` | `0.5` | SLAM scans merged for the distance (s) |
| `GO2_FEED_LIDAR_MIN_POINTS` | `3` | Smallest point cluster (distinct 5 cm voxels) taken as the person |

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
