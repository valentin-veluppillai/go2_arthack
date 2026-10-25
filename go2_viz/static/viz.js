// Human–robodog co-creation point cloud over the Go2 occupancy map.
//
// Two particle pools live on the map's surfaces:
//   * the base pool: a calm, sparse field everywhere (the room's skeleton)
//   * the focus pool: particles that only take spots inside the robot's view
//     cone; its size follows the human's movement (density ×1–15)
// Every particle is tied to one map point. A spring pulls it there, a slow flow
// field makes it drift, and now and then it hops to a nearby point. Particles
// never fly through empty space: when they need to go somewhere else they fade
// out and reappear at the new spot.
//
// Inputs (human distance, movement, entropy, posture confidence, scale; head
// yaw/pitch, cone angle) come from the panel or over ZMQ and are mapped to
//   distance   → jitter along surface normals, tearing below 1 m
//   movement   → focus density in the cone
//   entropy    → dispersion of focus particles away from the surfaces
//   confidence → saturation of the in-cone colouring (viridis, by distance)
//   scale      → share of particles drawn as diamond/crosshair markers
// Inside the cone two colour channels are screen-blended,
//   final = 1 − (1 − wr·robot)(1 − wh·human),  wh = engagement, wr = 1 − wh
// robot: time-of-flight false colour; human: the map point's photographic RGB
// (go2_map's camera colours, or a blurred test texture offline), weighted by
// how well that colour is known (alpha). Engagement rises with
// proximity and gesture velocity, so the human channel dominates up close.
// The human channel only shows where the human is: a soft vignette shaped by
// the detected bounding box (angles seen from the front camera) and limited in
// depth to around the human's distance. Without box inputs from go2_viz.feed
// the box of the simulated human column is used.
// Map coordinates are z-up; three.js is y-up.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import GUI from "lil-gui";

const BASE_MAX = 30000;              // slots [0, BASE_MAX) belong to the base pool
const FOCUS_MAX = 30000;             // slots [BASE_MAX, MAX_PARTICLES) to the focus pool
const MAX_PARTICLES = BASE_MAX + FOCUS_MAX;
const HOP_CELL = 0.4;     // m, neighbourhood for hops between map points
const RESEED = 0.1;       // share of base particles sent to a random spot of each new map
const MAX_HOP = 0.5;      // m, longer hops are skipped so particles don't cross gaps
const FADE_IN = 2.0;      // s, (re)appearing particles fade in from the background
const FADE_OUT = 0.6;     // s, leaving particles fade out
const CONE_REFRESH = 0.1; // s, how often the set of map points inside the cone is recomputed
const JITTER_RANGE = 0.06; // m, largest jitter offset along the normal at jitter = 1
const JITTER_RATE = 0.35;  // chance per 1/60 s that a particle re-rolls its jitter offset (×4 at jitter = 1)
const TEAR_RANGE = [0.15, 0.5]; // m, displacement of a tearing particle for one frame
const TEAR_CHANCE = 0.03;  // chance per 1/60 s that a particle tears at full tearing
const MAX_DISPERSION = 0.5; // m, how far focus particles drift from the surface at entropy = 1
const SATURATE = 0.4;      // s to reach full colour inside the field of view
const HEAD_OFFSET = new THREE.Vector3(0.3, 0.1, 0); // robot head in its own frame (three.js axes: x forward, y up)
const CAMERA_OFFSET = new THREE.Vector3(0.327, 0.043, 0); // front camera in the body frame (URDF), where box angles are seen from
const VIGNETTE_EDGE = [0.7, 1.3];  // box-normalised radius where the human channel starts / finishes fading out
const VIGNETTE_DEPTH = [0.4, 0.9]; // m from the human's distance where it starts / finishes fading out
const BOX_STALE = 2;               // s without box inputs before the simulated human's box takes over
const HUMAN_WIDTH = 0.5;           // m, width of the simulated human's box
// map frame (z up) → three.js frame (y up): -90° about x
const MAP_TO_THREE = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);

// ── inputs and their mapping to visual parameters ────────────────

const inputs = {
  distance: 5,       // human–robot distance (m)
  velocity: 0,       // human movement 0–1
  entropy: 0,        // interaction entropy 0–1
  confidence: 0,     // posture confidence 0–1
  scale: 0,          // perceived human scale 0–1
  humanBearing: 0,   // direction of the human relative to the body (deg, + = left)
  headYaw: 0,        // head yaw relative to the body (deg, + = left)
  headPitch: 0,      // head pitch (deg, + = up)
  coneAngle: 60,     // view cone, full angle (deg)
  trackHuman: false, // head follows the human
  boxLeft: 0, boxRight: 0, boxTop: 0, boxBottom: 0, // human bounding box seen from the camera (deg, + = left / up)
};
const INPUT_RANGES = {
  distance: [0.3, 5], velocity: [0, 1], entropy: [0, 1], confidence: [0, 1], scale: [0, 1],
  humanBearing: [-180, 180], headYaw: [-180, 180], headPitch: [-60, 60], coneAngle: [15, 90],
  boxLeft: [-90, 90], boxRight: [-90, 90], boxTop: [-90, 90], boxBottom: [-90, 90],
};

// visual parameters derived from the inputs every frame
const derived = { jitter: 0, tear: 0, focusFactor: 1, dispersion: 0, saturation: 0, markerRatio: 0, engagement: 0 };

function deriveFromInputs() {
  const d = inputs.distance;
  derived.jitter = THREE.MathUtils.clamp((1 / (d * d) - 1 / 9) / (1 - 1 / 9), 0, 1); // 1 at ≤1 m, 0 at ≥3 m
  derived.tear = THREE.MathUtils.clamp((1 - d) / 0.7, 0, 1);                          // 0 at 1 m, 1 at 0.3 m
  derived.focusFactor = 1 + 14 * inputs.velocity;
  derived.dispersion = inputs.entropy;
  derived.saturation = inputs.confidence;
  derived.markerRatio = inputs.scale;
  // either closeness (the jitter term: 1 at ≤1 m) or movement engages; together they saturate
  derived.engagement = 1 - (1 - derived.jitter) * (1 - inputs.velocity);
}

// tuning: the swarm's own behaviour; base case is calm, low-contrast grayscale
const params = {
  particles: 4000,    // base pool size
  focusDensity: 0.12, // focus particles per map point in the cone, at stillness (×1–15 with movement)
  attraction: 0.8,    // spring strength towards the target point
  damping: 0.95,      // velocity kept per 1/60 s
  noise: 0.12,        // flow field strength
  noiseScale: 0.5,    // flow field spatial frequency (1/m)
  flowSpeed: 0.08,    // flow field time scale
  wander: 0.004,      // chance per frame to hop to a neighbouring map point
  markerChange: 0.2,  // marker changes per second for each marker particle
  outlineChance: 0.3, // chance that a newly drawn diamond is outline only
  markerInflate: 3,   // markers are drawn up to this many times larger than circles (random per marker)
  floorShare: 0.15,   // share of base particles on the floor; the rest go to walls and objects
  size: 0.05,
  opacity: 0.55,
  ceiling: 1.8,       // hide map points this far above the floor (m)
  colorMode: "gray",
  paintMode: "time of flight", // in-cone colour: viridis by distance from the head, height colours, or one colour
  paintColor: "#ff8c1a",
  humanChannel: true, // screen-blend the human (photographic) channel into the cone paint
  fovRange: 4,        // m
  desaturate: 3,      // s for a particle to lose its colour after leaving the field of view
  showRobot: true,
  showFov: true,
  showHuman: true,
  autoRotate: true,
  rotateSpeed: 0.15,
};

const BACKGROUND = 0x1a1b1e;
const paintRGB = new THREE.Color(params.paintColor);

// Inputs from robot data arrive over the WebSocket (from go2_viz's ZMQ port 5561).
// Numbers ease towards the new value so noisy sensor data doesn't flicker.
const REMOTE_EASE = 0.5; // s, time constant
const remoteTargets = {};
let remoteUpdated = 0;   // performance.now() of the last remote update, 0 = never
let boxUpdated = 0;      // performance.now() of the last bounding box update, 0 = never

function applyRemoteInputs(update) {
  for (const [key, value] of Object.entries(update)) {
    if (key === "trackHuman") inputs.trackHuman = !!value;
    else if (key in INPUT_RANGES && Number.isFinite(value)) {
      const [lo, hi] = INPUT_RANGES[key];
      remoteTargets[key] = THREE.MathUtils.clamp(value, lo, hi);
      if (key.startsWith("box")) boxUpdated = performance.now();
    }
  }
  remoteUpdated = performance.now();
}

function easeRemoteInputs(dt) {
  const k = 1 - Math.exp(-dt / REMOTE_EASE);
  for (const [key, target] of Object.entries(remoteTargets)) {
    const [lo, hi] = INPUT_RANGES[key];
    inputs[key] += (target - inputs[key]) * k;
    if (Math.abs(target - inputs[key]) < (hi - lo) * 1e-4) { inputs[key] = target; delete remoteTargets[key]; }
  }
}

// ── renderer, camera ─────────────────────────────────────────────

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setClearColor(BACKGROUND, 1);
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.05, 500);
camera.position.set(8, 6, 8);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// ── particles ────────────────────────────────────────────────────

const positions = new Float32Array(MAX_PARTICLES * 3);   // simulated
const drawn = new Float32Array(MAX_PARTICLES * 3);       // simulated + jitter, rendered
const velocities = new Float32Array(MAX_PARTICLES * 3);
const colors = new Float32Array(MAX_PARTICLES * 3);
const alpha = new Float32Array(MAX_PARTICLES);           // fade in/out, rendered
const targetIndex = new Int32Array(MAX_PARTICLES);
const alive = new Uint8Array(MAX_PARTICLES);             // slot should be visible
const relocate = new Uint8Array(MAX_PARTICLES);          // focus particle fading out to reappear in the cone
const saturation = new Float32Array(MAX_PARTICLES);      // 0 = gray, 1 = full colour (field of view)
const jitterRoll = new Float32Array(MAX_PARTICLES);      // −1…1, offset along the normal
const dispersal = new Float32Array(MAX_PARTICLES * 3);   // baked dispersion direction × magnitude (focus pool)
// fixed per-particle roll: the particle is a marker while markerRoll < markerRatio,
// so a changing scale adds or removes markers instead of reshuffling all of them
const markerRoll = Float32Array.from({ length: MAX_PARTICLES }, Math.random);
const markerTile = new Float32Array(MAX_PARTICLES).fill(-1);
const inflate = new Float32Array(MAX_PARTICLES).fill(1);  // size factor: 1 for circles, random for markers

const geometry = new THREE.BufferGeometry();
geometry.setAttribute("position", new THREE.BufferAttribute(drawn, 3).setUsage(THREE.DynamicDrawUsage));
geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3).setUsage(THREE.DynamicDrawUsage));
geometry.setAttribute("alpha", new THREE.BufferAttribute(alpha, 1).setUsage(THREE.DynamicDrawUsage));
// atlas tile per particle: -1 = plain circle, otherwise 2 × marker index (+1 = outline only)
geometry.setAttribute("tile", new THREE.BufferAttribute(markerTile, 1).setUsage(THREE.DynamicDrawUsage));
geometry.setAttribute("inflate", new THREE.BufferAttribute(inflate, 1).setUsage(THREE.DynamicDrawUsage));
geometry.setDrawRange(0, MAX_PARTICLES);

// ── markers (after matplotlib's marker set) ──────────────────────

const MARKERS = ["o", "s", "^", "v", "<", ">", "D", "d", "p", "h", "8", "*", "P", "X", "+", "x"];
const LINE_ONLY = new Set(["+", "x"]); // drawn as strokes in both variants
const SCALE_MARKERS = ["D", "+"].map((m) => MARKERS.indexOf(m)); // high-alert shapes: diamond, crosshair
const TILE = 64, ATLAS_COLS = 8, ATLAS_ROWS = (MARKERS.length * 2) / ATLAS_COLS;

// closed path through n points on a circle; inner > 0 alternates with an inner radius (star)
function polygonPath(g, n, r, rot, inner = 0) {
  g.beginPath();
  const count = inner ? n * 2 : n;
  for (let k = 0; k < count; k++) {
    const rr = inner && k % 2 ? inner : r;
    const a = rot + (k / count) * Math.PI * 2;
    g.lineTo(rr * Math.cos(a), rr * Math.sin(a));
  }
  g.closePath();
}

// thick plus as a 12-corner outline (matplotlib "P"); rotate 45° for "X"
function plusPath(g, r, rot) {
  const w = r * 0.33;
  const pts = [[w, r], [w, w], [r, w], [r, -w], [w, -w], [w, -r], [-w, -r], [-w, -w], [-r, -w], [-r, w], [-w, w], [-w, r]];
  g.beginPath();
  for (const [x, y] of pts) g.lineTo(x * Math.cos(rot) - y * Math.sin(rot), x * Math.sin(rot) + y * Math.cos(rot));
  g.closePath();
}

// canvas y points down, so "up" is -90°
function drawMarker(g, m, r) {
  const up = -Math.PI / 2;
  switch (m) {
    case "o": g.beginPath(); g.arc(0, 0, r, 0, Math.PI * 2); break;
    case "s": polygonPath(g, 4, r, Math.PI / 4); break;
    case "^": polygonPath(g, 3, r, up); break;
    case "v": polygonPath(g, 3, r, -up); break;
    case "<": polygonPath(g, 3, r, Math.PI); break;
    case ">": polygonPath(g, 3, r, 0); break;
    case "D": polygonPath(g, 4, r, 0); break;
    case "d": g.save(); g.scale(0.6, 1); polygonPath(g, 4, r, 0); g.restore(); break;
    case "p": polygonPath(g, 5, r, up); break;
    case "h": polygonPath(g, 6, r, up); break;
    case "8": polygonPath(g, 8, r, Math.PI / 8); break;
    case "*": polygonPath(g, 5, r, up, r * 0.4); break;
    case "P": plusPath(g, r, 0); break;
    case "X": plusPath(g, r, Math.PI / 4); break;
    case "+": g.beginPath(); g.moveTo(-r, 0); g.lineTo(r, 0); g.moveTo(0, -r); g.lineTo(0, r); break;
    case "x": { const q = r * Math.SQRT1_2; g.beginPath(); g.moveTo(-q, -q); g.lineTo(q, q); g.moveTo(-q, q); g.lineTo(q, -q); break; }
  }
}

function markerAtlas() {
  const c = document.createElement("canvas");
  c.width = TILE * ATLAS_COLS;
  c.height = TILE * ATLAS_ROWS;
  const g = c.getContext("2d");
  g.fillStyle = g.strokeStyle = "#fff";
  g.lineJoin = "miter";
  g.miterLimit = 10;
  MARKERS.forEach((m, i) => {
    for (const outline of [false, true]) {
      const tile = i * 2 + (outline ? 1 : 0);
      g.save();
      g.translate((tile % ATLAS_COLS + 0.5) * TILE, (Math.floor(tile / ATLAS_COLS) + 0.5) * TILE);
      drawMarker(g, m, TILE * 0.36);  // padding keeps mipmaps from bleeding into neighbours
      if (outline || LINE_ONLY.has(m)) { g.lineWidth = TILE * 0.08; g.stroke(); } else g.fill();
      g.restore();
    }
  });
  const texture = new THREE.CanvasTexture(c);
  texture.anisotropy = 4;
  return texture;
}

const material = new THREE.ShaderMaterial({
  transparent: true,
  depthWrite: false,
  uniforms: {
    size: { value: params.size },
    scale: { value: 1 },  // half the drawing buffer height, for size attenuation like PointsMaterial
    minSize: { value: 2 },  // smallest point in device pixels, so far-away dots stay crisp instead of smearing
    opacity: { value: params.opacity },
    atlas: { value: markerAtlas() },
    grid: { value: new THREE.Vector2(ATLAS_COLS, ATLAS_ROWS) },
  },
  vertexShader: /* glsl */ `
    attribute vec3 color;
    attribute float alpha;
    attribute float tile;
    attribute float inflate;
    uniform float size;
    uniform float scale;
    uniform float minSize;
    varying vec3 vColor;
    varying float vAlpha;
    varying float vTile;
    void main() {
      vColor = color;
      vAlpha = alpha;
      vTile = tile;
      vec4 mv = modelViewMatrix * vec4(position, 1.0);
      gl_PointSize = alpha > 0.0 ? max(size * scale / -mv.z, minSize) * inflate : 0.0;
      gl_Position = projectionMatrix * mv;
    }`,
  fragmentShader: /* glsl */ `
    uniform float opacity;
    uniform sampler2D atlas;
    uniform vec2 grid;
    varying vec3 vColor;
    varying float vAlpha;
    varying float vTile;
    void main() {
      float shape;
      if (vTile < 0.0) {
        // crisp filled circle; the edge is smoothed over one pixel only, no blur
        float d = length(gl_PointCoord * 2.0 - 1.0);
        float aa = fwidth(d);
        shape = 1.0 - smoothstep(0.8 - aa, 0.8, d);
      } else {
        float col = mod(vTile, grid.x), row = floor(vTile / grid.x);
        vec2 uv = vec2((col + gl_PointCoord.x) / grid.x, 1.0 - (row + gl_PointCoord.y) / grid.y);
        shape = texture2D(atlas, uv).a;
      }
      float a = shape * vAlpha * opacity;
      if (a < 0.005) discard;
      gl_FragColor = vec4(vColor, a);
      #include <colorspace_fragment>
    }`,
});
const points = new THREE.Points(geometry, material);
// positions change every frame, so a bounding sphere computed once (around the origin, before the
// first map) would wrongly cull the whole cloud whenever the origin is out of view
points.frustumCulled = false;
scene.add(points);

function updatePointScale() {
  material.uniforms.scale.value = renderer.getDrawingBufferSize(new THREE.Vector2()).y / 2;
  material.uniforms.minSize.value = 2 * renderer.getPixelRatio();
}
updatePointScale();

// a diamond (sometimes outline only) or a crosshair
function randomMarkerTile() {
  const m = SCALE_MARKERS[Math.floor(Math.random() * SCALE_MARKERS.length)];
  return m * 2 + (!LINE_ONLY.has(MARKERS[m]) && Math.random() < params.outlineChance ? 1 : 0);
}

// ── robot, head, view cone, human ────────────────────────────────

const GRAY = 0x9a9da3;
const robot = new THREE.Group();
robot.add(new THREE.Mesh(new THREE.SphereGeometry(0.1, 16, 12), new THREE.MeshBasicMaterial({ color: GRAY })));
const head = new THREE.Object3D();      // yaw/pitch relative to the body; x forward
head.position.copy(HEAD_OFFSET);
head.rotation.order = "YZX";            // yaw about y first, then pitch about the turned z
robot.add(head);
const fovCone = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({
  color: GRAY, transparent: true, opacity: 0.06, depthWrite: false, side: THREE.DoubleSide,
}));
head.add(fovCone);
scene.add(robot);

let coneBuiltFor = { angle: 0, range: 0 };
function buildFovCone() {
  const angle = inputs.coneAngle, range = params.fovRange;
  if (Math.abs(angle - coneBuiltFor.angle) < 0.25 && range === coneBuiltFor.range) return;
  const half = THREE.MathUtils.degToRad(angle / 2);
  const cone = new THREE.ConeGeometry(range * Math.tan(half), range, 48, 1, true);
  cone.translate(0, -range / 2, 0); // apex at the head
  cone.rotateZ(Math.PI / 2);        // opening towards +x (forward)
  fovCone.geometry.dispose();
  fovCone.geometry = cone;
  coneBuiltFor = { angle, range };
}
buildFovCone();

// the simulated human: a translucent column at humanBearing / distance from the robot
const HUMAN_HEIGHT = 1.7;
const human = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, HUMAN_HEIGHT, 24, 1, true),
  new THREE.MeshBasicMaterial({ color: GRAY, transparent: true, opacity: 0.12, depthWrite: false, side: THREE.DoubleSide }));
scene.add(human);

// poses arrive once per second; the robot glides and turns towards the latest one.
// Without a pose (e.g. a PLY file) a virtual robot stands in the middle of the map.
const robotTarget = new THREE.Vector3();
const robotQuatTarget = new THREE.Quaternion();
let robotSource = "virtual";
const headPos = new THREE.Vector3(), headDir = new THREE.Vector3();
const headQuat = new THREE.Quaternion();
const look = { yaw: 0, pitch: 0 };      // eased head angles (rad)

function placeHuman() {
  const bearing = THREE.MathUtils.degToRad(inputs.humanBearing);
  const dir = new THREE.Vector3(Math.cos(bearing), 0, -Math.sin(bearing)).applyQuaternion(robot.quaternion);
  dir.y = 0;
  dir.normalize();
  human.position.copy(robot.position).addScaledVector(dir, inputs.distance);
  human.position.y = floorY + HUMAN_HEIGHT / 2;
  human.visible = params.showHuman;
}

function aimHead(dt) {
  let yaw = THREE.MathUtils.degToRad(inputs.headYaw), pitch = THREE.MathUtils.degToRad(inputs.headPitch);
  if (inputs.trackHuman) {
    // aim at the human's chest, in the body's frame
    const chest = human.position.clone().setY(floorY + 1.3);
    const v = robot.worldToLocal(chest).sub(HEAD_OFFSET);
    yaw = Math.atan2(-v.z, v.x);
    pitch = Math.atan2(v.y, Math.hypot(v.x, v.z));
  }
  const k = 1 - Math.exp(-dt * 4);
  look.yaw += (Math.atan2(Math.sin(yaw - look.yaw), Math.cos(yaw - look.yaw))) * k;  // shortest way round
  look.pitch += (pitch - look.pitch) * k;
  head.rotation.set(0, look.yaw, look.pitch);
  robot.updateMatrixWorld(true);
  head.getWorldPosition(headPos);
  head.getWorldQuaternion(headQuat);
  headDir.set(1, 0, 0).applyQuaternion(headQuat);
}

// the human's box as centre and half-size angles (rad) seen from the camera: from go2_viz.feed
// while it sends boxes, else the simulated human column's
const box = { az: 0, el: 0, halfAz: 0.1, halfEl: 0.1, source: "sim" };
const robotInverse = new THREE.Matrix4();

function updateHumanBox() {
  const rad = THREE.MathUtils.degToRad;
  let left, right, top, bottom;
  if (boxUpdated && performance.now() - boxUpdated < BOX_STALE * 1000) {
    [left, right, top, bottom] = [rad(inputs.boxLeft), rad(inputs.boxRight), rad(inputs.boxTop), rad(inputs.boxBottom)];
    box.source = "camera";
  } else {
    const c = robot.worldToLocal(human.position.clone()).sub(CAMERA_OFFSET);
    const az = Math.atan2(-c.z, c.x), d = Math.hypot(c.x, c.z), half = Math.atan2(HUMAN_WIDTH / 2, d);
    [left, right] = [az + half, az - half];
    [top, bottom] = [Math.atan2(c.y + HUMAN_HEIGHT / 2, d), Math.atan2(c.y - HUMAN_HEIGHT / 2, d)];
    box.source = "sim";
  }
  box.az = (left + right) / 2; box.halfAz = Math.max(Math.abs(left - right) / 2, rad(1));
  box.el = (top + bottom) / 2; box.halfEl = Math.max(Math.abs(top - bottom) / 2, rad(1));
  robotInverse.copy(robot.matrixWorld).invert();
}

// 0–1: how much a point (three.js world coords) lies where the human is
function humanMask(x, y, z) {
  const e = robotInverse.elements;
  const lx = e[0] * x + e[4] * y + e[8] * z + e[12];
  const ly = e[1] * x + e[5] * y + e[9] * z + e[13];
  const lz = e[2] * x + e[6] * y + e[10] * z + e[14];
  const depth = Math.abs(Math.hypot(lx, lz) - inputs.distance);
  if (depth >= VIGNETTE_DEPTH[1]) return 0;
  const cx = lx - CAMERA_OFFSET.x, cy = ly - CAMERA_OFFSET.y, cz = lz - CAMERA_OFFSET.z;
  let daz = Math.atan2(-cz, cx) - box.az;
  daz -= 2 * Math.PI * Math.round(daz / (2 * Math.PI)); // the simulated human can stand behind the robot
  const u = daz / box.halfAz, v = (Math.atan2(cy, Math.hypot(cx, cz)) - box.el) / box.halfEl;
  const r = Math.sqrt(Math.sqrt(u * u * u * u + v * v * v * v)); // rounded rectangle (superellipse) in angle space
  return (1 - THREE.MathUtils.smoothstep(r, VIGNETTE_EDGE[0], VIGNETTE_EDGE[1]))
       * (1 - THREE.MathUtils.smoothstep(depth, VIGNETTE_DEPTH[0], VIGNETTE_DEPTH[1]));
}

// ── map / targets ────────────────────────────────────────────────

let rawMap = null;                   // last map as received: Float32Array x, y, z (z-up)
let rawRGBA = null;                  // and its human-channel colours, Uint8Array r, g, b, a (or null)
let rawNormals = null;               // and its surface normals
let targets = new Float32Array(0);   // filtered map in three.js coords (y-up)
let targetNormals = new Float32Array(0);
let targetColors = new Float32Array(0);
let targetRGB = new Float32Array(0);  // human channel, 0–1
let targetAlpha = new Float32Array(0); // how well each human-channel colour is known, 0–1 (0 = no colour)
let mapHasRGB = false;
let targetCount = 0;
let structureCount = 0;              // targets [0, structureCount) are above the floor, the rest is floor
let targetCell = new Float64Array(0); // grid cell key of each target (exceeds int32)
let cellMembers = new Map();         // cell key → target indices in that cell (structure and floor kept apart)
let inCone = new Uint8Array(0);      // target lies inside the view cone
let coneList = [];                   // indices of targets inside the view cone
let floorY = 0;
let mapCenter = new THREE.Vector3();
let hasFitView = false;
let mapStats = { updates: 0, lastUpdate: 0, source: "–", points: 0 };

const PALETTE = [[40, 70, 220], [40, 190, 220], [80, 200, 80], [240, 220, 60], [230, 60, 40]];
const VIRIDIS = [[68, 1, 84], [59, 82, 139], [33, 145, 140], [94, 201, 98], [253, 231, 37]];

function ramp(stops, t, out, o) {
  const x = THREE.MathUtils.clamp(t, 0, 1) * (stops.length - 1);
  const i = Math.min(Math.floor(x), stops.length - 2), f = x - i;
  for (let c = 0; c < 3; c++) out[o + c] = (stops[i][c] * (1 - f) + stops[i + 1][c] * f) / 255;
}

function percentile(values, p) {
  const sorted = Float32Array.from(values).sort();
  return sorted.length ? sorted[Math.floor((sorted.length - 1) * p)] : 0;
}

function isFocus(i) { return i >= BASE_MAX; }

function rebuildTargets() {
  if (!rawMap) return;
  const n = rawMap.length / 3;
  const z = new Float32Array(n);
  for (let i = 0; i < n; i++) z[i] = rawMap[i * 3 + 2];
  const floor = percentile(z, 0.02);
  const top = Math.min(percentile(z, 0.98), floor + params.ceiling);
  floorY = floor;

  // structure (walls, objects) first, floor last; each part keeps the map's spatial order
  const keep = [], floorPoints = [];
  for (let i = 0; i < n; i++) {
    if (z[i] > floor + params.ceiling) continue;
    (z[i] < floor + 0.12 ? floorPoints : keep).push(i);
  }
  const oldTargets = targets, oldStructureCount = structureCount;
  structureCount = keep.length;
  keep.push(...floorPoints);
  targets = new Float32Array(keep.length * 3);
  targetNormals = new Float32Array(keep.length * 3);
  targetColors = new Float32Array(keep.length * 3);
  targetRGB = new Float32Array(keep.length * 3);
  targetAlpha = new Float32Array(keep.length);
  mapHasRGB = !!rawRGBA;
  keep.forEach((i, k) => {
    targets[k * 3] = rawMap[i * 3];             // x → x
    targets[k * 3 + 1] = rawMap[i * 3 + 2];     // z → y (up)
    targets[k * 3 + 2] = -rawMap[i * 3 + 1];    // y → -z
    targetNormals[k * 3] = rawNormals[i * 3];
    targetNormals[k * 3 + 1] = rawNormals[i * 3 + 2];
    targetNormals[k * 3 + 2] = -rawNormals[i * 3 + 1];
    ramp(PALETTE, (z[i] - floor) / Math.max(top - floor, 0.1), targetColors, k * 3);
    if (rawRGBA) {
      for (let c = 0; c < 3; c++) targetRGB[k * 3 + c] = rawRGBA[i * 4 + c] / 255;
      targetAlpha[k] = rawRGBA[i * 4 + 3] / 255;
    }
  });
  const first = targetCount === 0;
  targetCount = keep.length;
  inCone = new Uint8Array(targetCount);
  coneList = [];
  buildCells();
  if (!targetCount) return;
  const box = new THREE.Box3().setFromBufferAttribute(new THREE.BufferAttribute(targets, 3));
  box.getCenter(mapCenter);
  if (first) return; // slots are filled by step() once they are alive

  // the new map's points are in a different order: keep each particle where it is by
  // giving it a new target in the cell of its old one
  for (let i = 0; i < MAX_PARTICLES; i++) {
    if (!alive[i] && alpha[i] === 0) continue;
    const o = targetIndex[i] * 3;
    const members = !isFocus(i) && Math.random() < RESEED ? null :
      cellMembers.get(cellKey(oldTargets[o], oldTargets[o + 1], oldTargets[o + 2], targetIndex[i] >= oldStructureCount));
    if (members) targetIndex[i] = members[Math.floor(Math.random() * members.length)];
    else if (isFocus(i)) { targetIndex[i] = 0; relocate[i] = 1; }
    else respawn(i, pickTarget());
  }
}

// put a particle straight onto target t (no flight) and let it fade in
function respawn(i, t) {
  targetIndex[i] = t;
  const p = i * 3, q = t * 3;
  if (isFocus(i)) {
    // dispersion direction: mostly along the normal (either side), with some spread
    const s = Math.random() < 0.5 ? -1 : 1, m = Math.sqrt(Math.random()) * MAX_DISPERSION;
    const v = new THREE.Vector3(targetNormals[q] * s, targetNormals[q + 1] * s, targetNormals[q + 2] * s)
      .add(new THREE.Vector3().randomDirection().multiplyScalar(0.8)).normalize().multiplyScalar(m);
    dispersal[p] = v.x; dispersal[p + 1] = v.y; dispersal[p + 2] = v.z;
  }
  const spread = isFocus(i) ? derived.dispersion : 0;
  for (let c = 0; c < 3; c++) {
    positions[p + c] = targets[q + c] + dispersal[p + c] * spread + (Math.random() - 0.5) * 0.1;
    velocities[p + c] = 0;
  }
  alpha[i] = 0;
  saturation[i] = 0;
  relocate[i] = 0;
}

// random target, on the floor with probability floorShare
function pickTarget() {
  const floorCount = targetCount - structureCount;
  const onFloor = floorCount > 0 && (structureCount === 0 || Math.random() < params.floorShare);
  return onFloor ? structureCount + Math.floor(Math.random() * floorCount)
                 : Math.floor(Math.random() * structureCount);
}

function pickConeTarget() {
  return coneList.length ? coneList[Math.floor(Math.random() * coneList.length)] : -1;
}

// cell key from three.js coordinates; the floor gets its own key space so hops stay on their part
function cellKey(x, y, z, floor) {
  const ix = Math.floor(x / HOP_CELL) + 512, iy = Math.floor(y / HOP_CELL) + 512, iz = Math.floor(z / HOP_CELL) + 512;
  return ((ix * 1024 + iy) * 1024 + iz) * 2 + (floor ? 1 : 0);
}

function buildCells() {
  targetCell = new Float64Array(targetCount);
  cellMembers = new Map();
  for (let k = 0; k < targetCount; k++) {
    const key = cellKey(targets[k * 3], targets[k * 3 + 1], targets[k * 3 + 2], k >= structureCount);
    targetCell[k] = key;
    let members = cellMembers.get(key);
    if (!members) cellMembers.set(key, (members = []));
    members.push(k);
  }
}

// random target in the same or an adjacent cell of the same part (structure or floor),
// or ti itself if that target is more than MAX_HOP away
function neighbourTarget(ti) {
  const step = (Math.floor(Math.random() * 3) - 1) * 2 * 1024 * 1024    // x
             + (Math.floor(Math.random() * 3) - 1) * 2 * 1024           // y
             + (Math.floor(Math.random() * 3) - 1) * 2;                 // z
  const members = cellMembers.get(targetCell[ti] + step) || cellMembers.get(targetCell[ti]);
  const tj = members[Math.floor(Math.random() * members.length)];
  const dx = targets[tj * 3] - targets[ti * 3], dy = targets[tj * 3 + 1] - targets[ti * 3 + 1],
        dz = targets[tj * 3 + 2] - targets[ti * 3 + 2];
  return dx * dx + dy * dy + dz * dz <= MAX_HOP * MAX_HOP ? tj : ti;
}

// which map points are inside the view cone (refreshed every CONE_REFRESH)
function refreshCone() {
  const cosHalf = Math.cos(THREE.MathUtils.degToRad(inputs.coneAngle / 2));
  const range2 = params.fovRange * params.fovRange;
  coneList = [];
  for (let k = 0; k < targetCount; k++) {
    const dx = targets[k * 3] - headPos.x, dy = targets[k * 3 + 1] - headPos.y, dz = targets[k * 3 + 2] - headPos.z;
    const d2 = dx * dx + dy * dy + dz * dz;
    const along = dx * headDir.x + dy * headDir.y + dz * headDir.z;
    const inside = d2 < range2 && along > 0 && along * along >= cosHalf * cosHalf * d2;
    inCone[k] = inside ? 1 : 0;
    if (inside) coneList.push(k);
  }
}

function fitView() {
  if (!targetCount) return;
  const box = new THREE.Box3().setFromBufferAttribute(new THREE.BufferAttribute(targets, 3));
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length();
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(0.55, 0.45, 0.55).multiplyScalar(size));
  camera.near = size / 1000;
  camera.far = size * 20;
  camera.updateProjectionMatrix();
}

function onMap(header, xyz, normals, rgba) {
  rawMap = xyz;
  rawNormals = normals;
  rawRGBA = rgba;
  rebuildTargets();
  if (!hasFitView && targetCount) { fitView(); hasFitView = true; }
  if (header.robot) {
    const [x, y, z] = header.robot.position;
    const [qx, qy, qz, qw] = header.robot.orientation;
    robotTarget.set(x, z, -y);
    robotQuatTarget.copy(MAP_TO_THREE).multiply(new THREE.Quaternion(qx, qy, qz, qw))
                   .multiply(MAP_TO_THREE.clone().invert());
    if (robotSource !== "go2_map") { robot.position.copy(robotTarget); robot.quaternion.copy(robotQuatTarget); }
    robotSource = "go2_map";
  } else {
    robotSource = "virtual";
    robotTarget.set(mapCenter.x, floorY + 0.3, mapCenter.z);
    robotQuatTarget.identity();
    if (mapStats.updates === 0) robot.position.copy(robotTarget);
  }
  mapStats = { updates: mapStats.updates + 1, lastUpdate: performance.now(),
               source: header.source || "–", points: targetCount };
  document.getElementById("waiting").style.display = targetCount ? "none" : "grid";
}

// ── data link ────────────────────────────────────────────────────

let linkState = "off";
let ws = null;

function connect() {
  ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws/map`);
  ws.binaryType = "arraybuffer";
  ws.onopen = () => { linkState = "live"; };
  ws.onclose = () => { linkState = "off"; setTimeout(connect, 2000); };
  ws.onmessage = (ev) => {
    if (typeof ev.data === "string") { applyRemoteInputs(JSON.parse(ev.data).params || {}); return; }
    const view = new DataView(ev.data);
    const headerLen = view.getUint32(0, true);
    const header = JSON.parse(new TextDecoder().decode(new Uint8Array(ev.data, 4, headerLen)));
    const offset = 4 + headerLen + ((4 - ((4 + headerLen) % 4)) % 4);
    const n = header.count * 3;
    onMap(header, new Float32Array(ev.data, offset, n), new Float32Array(ev.data, offset + n * 4, n),
          header.rgba ? new Uint8Array(ev.data, offset + n * 8, header.count * 4) : null);
  };
}
connect();
setInterval(() => ws && ws.readyState === WebSocket.OPEN && ws.send("ping"), 10000);

// ── simulation ───────────────────────────────────────────────────

let coneTimer = 0;
let focusCount = 0;
let visibleFocus = 0;

function step(dt, time) {
  if (!targetCount) return;
  coneTimer -= dt;
  if (coneTimer <= 0) { refreshCone(); coneTimer = CONE_REFRESH; }

  // pool sizes: base is fixed, focus follows the cone's size and the human's movement
  const baseCount = Math.min(params.particles, BASE_MAX);
  focusCount = Math.min(FOCUS_MAX, Math.round(coneList.length * params.focusDensity * derived.focusFactor));
  for (let i = 0; i < MAX_PARTICLES; i++) alive[i] = isFocus(i) ? (i - BASE_MAX < focusCount ? 1 : 0) : (i < baseCount ? 1 : 0);

  const s = params.noiseScale, t = time * params.flowSpeed;
  const keep = Math.pow(params.damping, dt * 60);
  const hop = params.wander * dt * 60;
  const jitterAmp = derived.jitter * JITTER_RANGE, twitch = JITTER_RATE * (1 + 3 * derived.jitter) * dt * 60;
  const tearChance = derived.tear * TEAR_CHANCE * dt * 60;
  const spread = derived.dispersion;

  const cosHalf = Math.cos(THREE.MathUtils.degToRad(inputs.coneAngle / 2));
  const range2 = params.fovRange * params.fovRange;
  const gain = dt / SATURATE, loss = dt / params.desaturate;
  const fadeIn = dt / FADE_IN, fadeOut = dt / FADE_OUT;
  const markerSwitch = params.markerChange * dt;
  const conf = derived.saturation;
  const tof = params.paintMode === "time of flight", byHeight = params.paintMode === "height";
  const paint = [0, 0, 0];
  // dual stream: the human channel's weight, scaled per particle by the vignette around the human;
  // the robot channel gets the rest, so the screen never blows out
  const wh = params.humanChannel && mapHasRGB ? derived.engagement : 0;
  if (wh > 0) updateHumanBox();
  visibleFocus = 0;

  for (let i = 0; i < MAX_PARTICLES; i++) {
    const focus = isFocus(i);
    // appear, disappear, or relocate (fade out, then reappear inside the cone)
    if (alive[i] && !relocate[i]) {
      if (alpha[i] === 0) {
        const t0 = focus ? pickConeTarget() : pickTarget();
        if (t0 < 0) continue;
        respawn(i, t0);
      }
      alpha[i] = Math.min(1, alpha[i] + fadeIn);
    } else {
      alpha[i] = Math.max(0, alpha[i] - fadeOut);
      if (alpha[i] === 0) {
        if (relocate[i] && alive[i]) { relocate[i] = 0; const t0 = pickConeTarget(); if (t0 >= 0) respawn(i, t0); }
        continue;
      }
    }
    if (focus) visibleFocus++;

    const p = i * 3;
    let ti = targetIndex[i];
    if (Math.random() < hop) {
      // mostly hop to a nearby point, sometimes reappear somewhere else
      if (focus) {
        const tj = neighbourTarget(ti);
        if (inCone[tj]) targetIndex[i] = tj;
      } else if (Math.random() < 0.02) respawn(i, pickTarget());
      else targetIndex[i] = neighbourTarget(ti);
      ti = targetIndex[i];
    }
    if (focus && !inCone[ti] && !relocate[i]) relocate[i] = 1;  // the cone moved away: fade out, reappear inside

    const q = ti * 3;
    const px = positions[p], py = positions[p + 1], pz = positions[p + 2];
    const fx = Math.sin(py * s + t) + Math.cos(pz * s * 1.7 - t);
    const fy = Math.sin(pz * s + t * 1.3) + Math.cos(px * s * 1.3 + t);
    const fz = Math.sin(px * s - t * 0.7) + Math.cos(py * s * 1.5 + t * 0.6);
    // focus particles are pulled to their spot pushed off the surface by the dispersion
    const ox = focus ? dispersal[p] * spread : 0, oy = focus ? dispersal[p + 1] * spread : 0, oz = focus ? dispersal[p + 2] * spread : 0;

    let vx = velocities[p] + ((targets[q] + ox - px) * params.attraction + fx * params.noise) * dt;
    let vy = velocities[p + 1] + ((targets[q + 1] + oy - py) * params.attraction + fy * params.noise) * dt;
    let vz = velocities[p + 2] + ((targets[q + 2] + oz - pz) * params.attraction + fz * params.noise) * dt;
    vx *= keep; vy *= keep; vz *= keep;
    velocities[p] = vx; velocities[p + 1] = vy; velocities[p + 2] = vz;
    positions[p] = px + vx * dt; positions[p + 1] = py + vy * dt; positions[p + 2] = pz + vz * dt;

    // jitter: vibration along the surface normal; below 1 m some particles tear off for a frame
    let off = 0;
    if (jitterAmp > 0) {
      if (Math.random() < twitch) jitterRoll[i] = Math.random() * 2 - 1;
      off = jitterRoll[i] * jitterAmp;
    }
    if (tearChance > 0 && Math.random() < tearChance) {
      off += (Math.random() < 0.5 ? -1 : 1) * (TEAR_RANGE[0] + Math.random() * (TEAR_RANGE[1] - TEAR_RANGE[0]));
    }
    drawn[p] = positions[p] + targetNormals[q] * off;
    drawn[p + 1] = positions[p + 1] + targetNormals[q + 1] * off;
    drawn[p + 2] = positions[p + 2] + targetNormals[q + 2] * off;

    // field of view: colour builds up inside the cone and decays outside
    const dx = drawn[p] - headPos.x, dy = drawn[p + 1] - headPos.y, dz = drawn[p + 2] - headPos.z;
    const d2 = dx * dx + dy * dy + dz * dz;
    const along = dx * headDir.x + dy * headDir.y + dz * headDir.z;
    const inside = d2 < range2 && along > 0 && along * along >= cosHalf * cosHalf * d2;
    const sat = saturation[i] = inside ? Math.min(1, saturation[i] + gain) : Math.max(0, saturation[i] - loss);

    if (markerRoll[i] >= derived.markerRatio) { markerTile[i] = -1; inflate[i] = 1; }
    else if (markerTile[i] < 0 || Math.random() < markerSwitch) {
      markerTile[i] = randomMarkerTile();
      // every new marker gets its own size; squared, so most stay modest and a few are large
      const r = Math.random();
      inflate[i] = 1 + (params.markerInflate - 1) * r * r;
    }

    const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
    const glow = 0.45 + Math.min(speed * 0.6, 1.2);
    if (params.colorMode === "gray") {
      // narrow band of mid grays: slightly lighter higher up and when moving
      const h = (targetColors[q] + targetColors[q + 1] + targetColors[q + 2]) / 3;
      const g = 0.42 + 0.08 * h + Math.min(speed * 0.15, 0.06);
      if (sat > 0) {
        // paint: viridis by distance from the head (faked time of flight), height colours or one colour,
        // desaturated towards its own luminance when posture confidence is low
        if (tof) ramp(VIRIDIS, Math.sqrt(d2) / params.fovRange, paint, 0);
        else if (byHeight) { paint[0] = targetColors[q] * 0.9; paint[1] = targetColors[q + 1] * 0.9; paint[2] = targetColors[q + 2] * 0.9; }
        else { paint[0] = paintRGB.r; paint[1] = paintRGB.g; paint[2] = paintRGB.b; }
        const w = wh > 0 && targetAlpha[ti] > 0 ? wh * targetAlpha[ti] * humanMask(drawn[p], drawn[p + 1], drawn[p + 2]) : 0;
        if (w > 0) for (let c = 0; c < 3; c++) paint[c] = 1 - (1 - (1 - w) * paint[c]) * (1 - w * targetRGB[q + c]);
        const lum = 0.2126 * paint[0] + 0.7152 * paint[1] + 0.0722 * paint[2];
        for (let c = 0; c < 3; c++) colors[p + c] = g + (lum + (paint[c] - lum) * conf - g) * sat;
      } else {
        colors[p] = colors[p + 1] = colors[p + 2] = g;
      }
    } else if (params.colorMode === "height") {
      colors[p] = targetColors[q] * glow; colors[p + 1] = targetColors[q + 1] * glow; colors[p + 2] = targetColors[q + 2] * glow;
    } else if (params.colorMode === "speed") {
      ramp(PALETTE, Math.min(speed / 1.5, 1), colors, p);
    } else {
      colors[p] = colors[p + 1] = colors[p + 2] = 0.55 * glow;
    }
  }
  geometry.attributes.position.needsUpdate = true;
  geometry.attributes.color.needsUpdate = true;
  geometry.attributes.alpha.needsUpdate = true;
  geometry.attributes.tile.needsUpdate = true;
  geometry.attributes.inflate.needsUpdate = true;
}

// ── UI ───────────────────────────────────────────────────────────

const gui = new GUI({ title: "Co-creation" });

const humanFolder = gui.addFolder("Human");
humanFolder.add(inputs, "distance", 0.3, 5, 0.01).name("distance (m)");
humanFolder.add(inputs, "velocity", 0, 1, 0.01).name("gesture velocity");
humanFolder.add(inputs, "confidence", 0, 1, 0.01).name("posture confidence");
humanFolder.add(inputs, "scale", 0, 1, 0.01).name("detected scale");
humanFolder.add(inputs, "humanBearing", -180, 180, 1).name("bearing (°)");
humanFolder.add(params, "showHuman").name("show human");

const robotFolder = gui.addFolder("Robot & canvas");
robotFolder.add(inputs, "headYaw", -180, 180, 1).name("head yaw (°)");
robotFolder.add(inputs, "headPitch", -60, 60, 1).name("head pitch (°)");
robotFolder.add(inputs, "coneAngle", 15, 90, 1).name("cone angle (°)");
robotFolder.add(inputs, "trackHuman").name("head tracks human");
robotFolder.add(inputs, "entropy", 0, 1, 0.01).name("entropy / noise");
robotFolder.add(params, "showRobot").name("show robot").onChange((v) => { robot.children[0].visible = v; });
robotFolder.add(params, "showFov").name("show view cone").onChange((v) => { fovCone.visible = v; });

const tuning = gui.addFolder("Tuning");
tuning.close();
const swarm = tuning.addFolder("Swarm");
swarm.add(params, "particles", 0, BASE_MAX, 500).name("base particles");
swarm.add(params, "focusDensity", 0, 0.5, 0.005).name("focus density");
swarm.add(params, "attraction", 0, 12, 0.1);
swarm.add(params, "damping", 0.7, 0.99, 0.01);
swarm.add(params, "noise", 0, 5, 0.05).name("flow drift");
swarm.add(params, "noiseScale", 0.05, 4, 0.05).name("flow scale");
swarm.add(params, "flowSpeed", 0, 3, 0.05).name("flow speed");
swarm.add(params, "wander", 0, 0.3, 0.005);
swarm.add(params, "floorShare", 0, 1, 0.01).name("floor share");
const lookFolder = tuning.addFolder("Look");
lookFolder.add(params, "size", 0.005, 0.2, 0.005).onChange((v) => { material.uniforms.size.value = v; });
lookFolder.add(params, "opacity", 0.05, 1, 0.01).onChange((v) => { material.uniforms.opacity.value = v; });
lookFolder.add(params, "colorMode", ["gray", "height", "speed", "white"]);
lookFolder.add(params, "ceiling", 0.3, 5, 0.1).name("ceiling (m)").onChange(rebuildTargets);
lookFolder.add(params, "paintMode", ["time of flight", "height", "color"]).name("cone paint");
lookFolder.addColor(params, "paintColor").name("paint colour").onChange((v) => paintRGB.set(v));
lookFolder.add(params, "humanChannel").name("human channel");
lookFolder.add(params, "fovRange", 0.5, 10, 0.1).name("view range (m)");
lookFolder.add(params, "desaturate", 0.2, 10, 0.1).name("desaturate (s)");
const markerFolder = tuning.addFolder("Markers");
markerFolder.add(params, "markerChange", 0, 5, 0.05).name("changes / s");
markerFolder.add(params, "outlineChance", 0, 1, 0.01).name("outline chance");
markerFolder.add(params, "markerInflate", 1, 6, 0.1).name("max inflation");
const cam = tuning.addFolder("Camera");
cam.add(params, "autoRotate");
cam.add(params, "rotateSpeed", -3, 3, 0.1);
cam.add({ fit: fitView }, "fit").name("fit view");
for (const f of [swarm, lookFolder, markerFolder, cam]) f.close();

document.getElementById("snapshot").onclick = () => {
  renderer.domElement.toBlob((blob) => download(blob, `go2_swarm_${Date.now()}.png`));
};

let recorder = null;
document.getElementById("record").onclick = (ev) => {
  const button = ev.currentTarget;
  if (recorder) { recorder.stop(); return; }
  const chunks = [];
  recorder = new MediaRecorder(renderer.domElement.captureStream(30), { mimeType: "video/webm" });
  recorder.ondataavailable = (e) => chunks.push(e.data);
  recorder.onstop = () => {
    download(new Blob(chunks, { type: "video/webm" }), `go2_swarm_${Date.now()}.webm`);
    recorder = null; button.textContent = "Record"; button.classList.remove("recording");
  };
  recorder.start();
  button.textContent = "Stop"; button.classList.add("recording");
};

function download(blob, name) {
  const a = Object.assign(document.createElement("a"), { href: URL.createObjectURL(blob), download: name });
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

const hud = Object.fromEntries(["source", "link", "points", "updates", "age", "robotsrc", "cone", "focus", "mapping", "remote", "fps"]
  .map((id) => [id, document.getElementById(id)]));
let frames = 0, fpsTime = performance.now();

function updateHud(now) {
  frames++;
  if (now - fpsTime < 500) return;
  hud.fps.textContent = Math.round((frames * 1000) / (now - fpsTime));
  frames = 0; fpsTime = now;
  const age = mapStats.updates ? (now - mapStats.lastUpdate) / 1000 : null;
  hud.source.textContent = mapStats.source;
  hud.points.textContent = mapStats.points.toLocaleString();
  hud.updates.textContent = mapStats.updates;
  hud.age.textContent = age === null ? "–" : `${age.toFixed(1)} s ago`;
  hud.link.textContent = linkState === "live" ? (age !== null && age > 5 ? "connected, idle" : "live") : "reconnecting";
  hud.link.className = linkState === "live" ? (age !== null && age > 5 ? "stale" : "live") : "off";
  hud.robotsrc.textContent = robotSource;
  hud.cone.textContent = coneList.length.toLocaleString();
  hud.focus.textContent = `${visibleFocus.toLocaleString()} (×${derived.focusFactor.toFixed(1)})`;
  hud.mapping.textContent = `jitter ${derived.jitter.toFixed(2)}${derived.tear > 0 ? " tear" : ""} · disp ${derived.dispersion.toFixed(2)} · sat ${derived.saturation.toFixed(2)} · markers ${derived.markerRatio.toFixed(2)}${mapHasRGB ? ` · human ${derived.engagement.toFixed(2)} (${box.source} box)` : ""}`;
  hud.remote.textContent = remoteUpdated ? `${((now - remoteUpdated) / 1000).toFixed(1)} s ago` : "–";
  // keep the panel in sync while robot data is moving the sliders
  if (remoteUpdated && now - remoteUpdated < REMOTE_EASE * 10 * 1000) {
    gui.controllersRecursive().forEach((c) => c.updateDisplay());
  }
}

// ── loop ─────────────────────────────────────────────────────────

const clock = new THREE.Clock();

function frame() {
  const dt = Math.min(clock.getDelta(), 1 / 20);
  easeRemoteInputs(dt);
  deriveFromInputs();
  robot.position.lerp(robotTarget, 1 - Math.exp(-dt * 3));
  robot.quaternion.slerp(robotQuatTarget, 1 - Math.exp(-dt * 3));
  placeHuman();
  aimHead(dt);
  buildFovCone();
  step(dt, clock.elapsedTime);
  controls.autoRotate = params.autoRotate;
  controls.autoRotateSpeed = params.rotateSpeed;
  controls.update();

  renderer.render(scene, camera);
  updateHud(performance.now());
  requestAnimationFrame(frame);
}
frame();

// handle for tweaking from the browser console, e.g. swarm.inputs.distance = 0.6
window.swarm = { inputs, derived, params, robot, head, human, fovCone, camera, controls, material };

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
  updatePointScale();
});
