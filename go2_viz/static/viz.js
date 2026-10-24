// Particle swarm over the Go2 occupancy map.
//
// Every particle is tied to one map point (its target). A spring pulls it
// there, a time-varying flow field makes it drift around it, and now and then
// it hops to a nearby map point, so the swarm crawls over the surfaces.
// Particles that need to go somewhere far away never fly through empty
// space: they are respawned at the new spot and fade in from the background.
// When a new map arrives, targets are swapped in place and the swarm flows to
// the new geometry. Map coordinates are z-up; three.js is y-up.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import GUI from "lil-gui";

const MAX_PARTICLES = 60000;
const HOP_CELL = 0.4; // m, neighbourhood for hops between map points
const RESEED = 0.1;   // share of particles sent to a random spot of each new map, so new areas fill up
const MAX_HOP = 0.5;  // m, longer hops are skipped so particles don't cross gaps
const FADE_IN = 2.0;  // s, respawned particles fade in from the background

// base case: calm, low-contrast grayscale, few slow particles
const params = {
  particles: 4000,
  attraction: 0.8,    // spring strength towards the target point
  damping: 0.95,      // velocity kept per 1/60 s
  noise: 0.12,        // flow field strength
  noiseScale: 0.5,    // flow field spatial frequency (1/m)
  flowSpeed: 0.08,    // flow field time scale
  wander: 0.004,      // chance per frame to hop to a neighbouring map point
  floorShare: 0.15,   // share of particles on the floor; the rest go to walls and objects
  size: 0.05,
  opacity: 0.55,
  ceiling: 1.8,       // hide map points this far above the floor (m)
  colorMode: "gray",
  autoRotate: true,
  rotateSpeed: 0.15,
  showRobot: true,
};

const BACKGROUND = 0x1a1b1e;
const BG = new THREE.Color(BACKGROUND); // linear RGB, same space as the vertex colours

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

const positions = new Float32Array(MAX_PARTICLES * 3);
const velocities = new Float32Array(MAX_PARTICLES * 3);
const colors = new Float32Array(MAX_PARTICLES * 3);
const targetIndex = new Int32Array(MAX_PARTICLES);
const fade = new Float32Array(MAX_PARTICLES).fill(1); // 0 = background colour, 1 = fully visible

const geometry = new THREE.BufferGeometry();
geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3).setUsage(THREE.DynamicDrawUsage));
geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3).setUsage(THREE.DynamicDrawUsage));
geometry.setDrawRange(0, 0);

function discTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 64;
  const g = c.getContext("2d");
  const grad = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grad.addColorStop(0, "rgba(255,255,255,1)");
  grad.addColorStop(0.35, "rgba(255,255,255,0.6)");
  grad.addColorStop(1, "rgba(255,255,255,0)");
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(c);
}

const material = new THREE.PointsMaterial({
  size: params.size, map: discTexture(), vertexColors: true, transparent: true,
  opacity: params.opacity, depthWrite: false, sizeAttenuation: true,
});
scene.add(new THREE.Points(geometry, material));

const robot = new THREE.Mesh(new THREE.SphereGeometry(0.1, 16, 12),
                             new THREE.MeshBasicMaterial({ color: 0x9a9da3 }));
robot.visible = false;
scene.add(robot);
const robotTarget = new THREE.Vector3(); // poses arrive once per second; the marker glides there

// ── map / targets ────────────────────────────────────────────────

let rawMap = null;               // last map as received: Float32Array x, y, z (z-up)
let targets = new Float32Array(0); // filtered map in three.js coords (y-up)
let targetColors = new Float32Array(0);
let targetCount = 0;
let structureCount = 0;          // targets [0, structureCount) are above the floor, the rest is floor
let targetCell = new Float64Array(0); // grid cell key of each target (exceeds int32)
let cellMembers = new Map();        // cell key → target indices in that cell (structure and floor kept apart)
let hasFitView = false;
let mapStats = { updates: 0, lastUpdate: 0, source: "–", points: 0 };

const PALETTE = [[40, 70, 220], [40, 190, 220], [80, 200, 80], [240, 220, 60], [230, 60, 40]];

function heightColor(t, out, o) {
  const x = Math.min(Math.max(t, 0), 1) * (PALETTE.length - 1);
  const i = Math.min(Math.floor(x), PALETTE.length - 2), f = x - i;
  for (let c = 0; c < 3; c++) out[o + c] = (PALETTE[i][c] * (1 - f) + PALETTE[i + 1][c] * f) / 255;
}

function percentile(values, p) {
  const sorted = Float32Array.from(values).sort();
  return sorted.length ? sorted[Math.floor((sorted.length - 1) * p)] : 0;
}

function rebuildTargets() {
  if (!rawMap) return;
  const n = rawMap.length / 3;
  const z = new Float32Array(n);
  for (let i = 0; i < n; i++) z[i] = rawMap[i * 3 + 2];
  const floor = percentile(z, 0.02);
  const top = Math.min(percentile(z, 0.98), floor + params.ceiling);

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
  targetColors = new Float32Array(keep.length * 3);
  keep.forEach((i, k) => {
    targets[k * 3] = rawMap[i * 3];           // x → x
    targets[k * 3 + 1] = rawMap[i * 3 + 2];   // z → y (up)
    targets[k * 3 + 2] = -rawMap[i * 3 + 1];  // y → -z
    heightColor((z[i] - floor) / Math.max(top - floor, 0.1), targetColors, k * 3);
  });
  const first = targetCount === 0;
  targetCount = keep.length;
  buildCells();
  if (!targetCount) return;
  if (first) { scatterParticles(0, MAX_PARTICLES); return; }

  // the new map's points are in a different order: keep each particle where it is by
  // giving it a new target in the cell of its old one
  for (let i = 0; i < MAX_PARTICLES; i++) {
    const o = targetIndex[i] * 3;
    const members = Math.random() < RESEED ? null :
      cellMembers.get(cellKey(oldTargets[o], oldTargets[o + 1], oldTargets[o + 2], targetIndex[i] >= oldStructureCount));
    if (members) targetIndex[i] = members[Math.floor(Math.random() * members.length)];
    else respawn(i, pickTarget());
  }
}

// put a particle straight onto target t (no flight) and let it fade in
function respawn(i, t) {
  targetIndex[i] = t;
  for (let c = 0; c < 3; c++) {
    positions[i * 3 + c] = targets[t * 3 + c] + (Math.random() - 0.5) * 0.1;
    velocities[i * 3 + c] = 0;
  }
  fade[i] = 0;
}

// random target, on the floor with probability floorShare
function pickTarget() {
  const floorCount = targetCount - structureCount;
  const onFloor = floorCount > 0 && (structureCount === 0 || Math.random() < params.floorShare);
  return onFloor ? structureCount + Math.floor(Math.random() * floorCount)
                 : Math.floor(Math.random() * structureCount);
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

function scatterParticles(from, to) {
  for (let i = from; i < to; i++) respawn(i, pickTarget());
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

function onMap(header, xyz) {
  rawMap = xyz;
  rebuildTargets();
  if (!hasFitView && targetCount) { fitView(); hasFitView = true; }
  if (header.robot) {
    const [x, y, z] = header.robot.position;
    robotTarget.set(x, z, -y);
    if (!robot.visible) robot.position.copy(robotTarget);
  }
  robot.visible = params.showRobot && !!header.robot;
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
    const view = new DataView(ev.data);
    const headerLen = view.getUint32(0, true);
    const header = JSON.parse(new TextDecoder().decode(new Uint8Array(ev.data, 4, headerLen)));
    const offset = 4 + headerLen + ((4 - ((4 + headerLen) % 4)) % 4);
    onMap(header, new Float32Array(ev.data, offset, header.count * 3));
  };
}
connect();
setInterval(() => ws && ws.readyState === WebSocket.OPEN && ws.send("ping"), 10000);

// ── simulation ───────────────────────────────────────────────────

function step(dt, time) {
  const n = Math.min(params.particles, MAX_PARTICLES);
  geometry.setDrawRange(0, targetCount ? n : 0);
  if (!targetCount) return;

  const s = params.noiseScale, t = time * params.flowSpeed;
  const keep = Math.pow(params.damping, dt * 60);
  const hop = params.wander * dt * 60;

  for (let i = 0; i < n; i++) {
    const p = i * 3;
    let ti = targetIndex[i];
    if (Math.random() < hop) {
      // mostly hop to a nearby point, sometimes reappear somewhere else
      if (Math.random() < 0.02) respawn(i, pickTarget());
      else targetIndex[i] = neighbourTarget(ti);
      ti = targetIndex[i];
    }
    const q = ti * 3;
    const px = positions[p], py = positions[p + 1], pz = positions[p + 2];

    const fx = Math.sin(py * s + t) + Math.cos(pz * s * 1.7 - t);
    const fy = Math.sin(pz * s + t * 1.3) + Math.cos(px * s * 1.3 + t);
    const fz = Math.sin(px * s - t * 0.7) + Math.cos(py * s * 1.5 + t * 0.6);

    let vx = velocities[p] + ((targets[q] - px) * params.attraction + fx * params.noise) * dt;
    let vy = velocities[p + 1] + ((targets[q + 1] - py) * params.attraction + fy * params.noise) * dt;
    let vz = velocities[p + 2] + ((targets[q + 2] - pz) * params.attraction + fz * params.noise) * dt;
    vx *= keep; vy *= keep; vz *= keep;
    velocities[p] = vx; velocities[p + 1] = vy; velocities[p + 2] = vz;
    positions[p] = px + vx * dt; positions[p + 1] = py + vy * dt; positions[p + 2] = pz + vz * dt;

    const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
    const glow = 0.45 + Math.min(speed * 0.6, 1.2);
    if (params.colorMode === "height") {
      colors[p] = targetColors[q] * glow; colors[p + 1] = targetColors[q + 1] * glow; colors[p + 2] = targetColors[q + 2] * glow;
    } else if (params.colorMode === "gray") {
      // narrow band of mid grays: slightly lighter higher up and when moving
      const h = (targetColors[q] + targetColors[q + 1] + targetColors[q + 2]) / 3;
      colors[p] = colors[p + 1] = colors[p + 2] = 0.42 + 0.08 * h + Math.min(speed * 0.15, 0.06);
    } else if (params.colorMode === "speed") {
      heightColor(Math.min(speed / 1.5, 1), colors, p);
    } else {
      colors[p] = colors[p + 1] = colors[p + 2] = 0.55 * glow;
    }
    if (fade[i] < 1) {
      const f = fade[i];
      colors[p] = BG.r + (colors[p] - BG.r) * f;
      colors[p + 1] = BG.g + (colors[p + 1] - BG.g) * f;
      colors[p + 2] = BG.b + (colors[p + 2] - BG.b) * f;
      fade[i] = Math.min(1, f + dt / FADE_IN);
    }
  }
  geometry.attributes.position.needsUpdate = true;
  geometry.attributes.color.needsUpdate = true;
}

// ── UI ───────────────────────────────────────────────────────────

const gui = new GUI({ title: "Swarm" });
const swarm = gui.addFolder("Swarm");
swarm.add(params, "particles", 1000, MAX_PARTICLES, 1000);
swarm.add(params, "attraction", 0, 12, 0.1);
swarm.add(params, "damping", 0.7, 0.99, 0.01);
swarm.add(params, "noise", 0, 5, 0.05);
swarm.add(params, "noiseScale", 0.05, 4, 0.05);
swarm.add(params, "flowSpeed", 0, 3, 0.05);
swarm.add(params, "wander", 0, 0.3, 0.005);
swarm.add(params, "floorShare", 0, 1, 0.01).name("floor share").onChange(() => scatterParticles(0, MAX_PARTICLES));
const look = gui.addFolder("Look");
look.add(params, "size", 0.005, 0.2, 0.005).onChange((v) => { material.size = v; });
look.add(params, "opacity", 0.05, 1, 0.01).onChange((v) => { material.opacity = v; });
look.add(params, "colorMode", ["gray", "height", "speed", "white"]);
look.add(params, "ceiling", 0.3, 5, 0.1).name("ceiling (m)").onChange(rebuildTargets);
look.add(params, "showRobot").onChange((v) => { robot.visible = v && mapStats.updates > 0; });
const cam = gui.addFolder("Camera");
cam.add(params, "autoRotate");
cam.add(params, "rotateSpeed", -3, 3, 0.1);
cam.add({ fit: fitView }, "fit").name("fit view");

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

const hud = Object.fromEntries(["source", "link", "points", "updates", "age", "fps"].map((id) => [id, document.getElementById(id)]));
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
}

// ── loop ─────────────────────────────────────────────────────────

const clock = new THREE.Clock();

function frame() {
  const dt = Math.min(clock.getDelta(), 1 / 20);
  step(dt, clock.elapsedTime);
  robot.position.lerp(robotTarget, 1 - Math.exp(-dt * 3));
  controls.autoRotate = params.autoRotate;
  controls.autoRotateSpeed = params.rotateSpeed;
  controls.update();

  renderer.render(scene, camera);
  updateHud(performance.now());
  requestAnimationFrame(frame);
}
frame();

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
