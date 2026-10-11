// Tercio S1 · NEMA 23 adapter viewer. Units: mm, Y up (the CAD Z axis), Y = 0 on the motor's rear face.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

const layout = await (await fetch('./models/layout.json')).json();
const stage = document.getElementById('stage');

// ---- renderer ---------------------------------------------------------------------------------------
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, stencil: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.toneMapping = THREE.NeutralToneMapping;
renderer.toneMappingExposure = 1.0;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.localClippingEnabled = true;
stage.appendChild(renderer.domElement);
stage.style.background = 'radial-gradient(120% 90% at 62% 42%, #fbfbfc 0%, #f1f2f5 55%, #e2e5ea 100%)';

const labels = new CSS2DRenderer();
labels.setSize(innerWidth, innerHeight);
Object.assign(labels.domElement.style, { position: 'fixed', inset: '0', pointerEvents: 'none' });
document.body.appendChild(labels.domElement);

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.85;

const camera = new THREE.PerspectiveCamera(30, innerWidth / innerHeight, 1, 5000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, -18, 0);

// Lights: a soft key with shadows, a cool rim, a sky/ground fill.
const key = new THREE.DirectionalLight(0xffffff, 2.2);
key.position.set(-160, 260, 180);
key.castShadow = true;
key.shadow.mapSize.set(4096, 4096);
Object.assign(key.shadow.camera, { left: -140, right: 140, top: 140, bottom: -140, near: 50, far: 800 });
key.shadow.radius = 6;
key.shadow.bias = -0.0002;
key.shadow.normalBias = 0.02;
scene.add(key);
const rim = new THREE.DirectionalLight(0xdfe6ff, 0.9);
rim.position.set(200, 80, -220);
scene.add(rim);
scene.add(new THREE.HemisphereLight(0xffffff, 0xd9dde5, 0.45));

const ground = new THREE.Mesh(new THREE.PlaneGeometry(2000, 2000), new THREE.ShadowMaterial({ opacity: 0.16, color: 0x1c2233 }));
ground.rotation.x = -Math.PI / 2;
ground.position.y = -78;
ground.receiveShadow = true;
scene.add(ground);

// ---- materials --------------------------------------------------------------------------------------
const clip = new THREE.Plane(new THREE.Vector3(0, 0, -1), 0);
const phys = (color, metalness, roughness, extra = {}) =>
  new THREE.MeshPhysicalMaterial({ color, metalness, roughness, clippingPlanes: [], ...extra });
const MATS = {
  adapter: phys('#c8cbd0', 1.0, 0.42),
  magnet_n: phys('#cf3a36', 0.2, 0.45),
  magnet_s: phys('#b4b9c1', 0.85, 0.32),
  plate_screws: phys('#16171a', 0.7, 0.35),
  s1_screws: phys('#16171a', 0.7, 0.35),
  motor_front_cap: phys('#232428', 0.55, 0.5),
  motor_rear_cap: phys('#232428', 0.55, 0.5),
  motor_pilot: phys('#2b2c31', 0.6, 0.42),
  motor_stator: phys('#2a2c30', 0.8, 0.36),
  motor_bearing: phys('#9ea3aa', 1.0, 0.3),
  motor_shaft: phys('#d3d6da', 1.0, 0.2),
  motor_screws: phys('#a7abb1', 1.0, 0.3),
  motor_lead_exit: phys('#0c0c0d', 0.0, 0.75),
  motor_lead_black: phys('#141414', 0.0, 0.5),
  motor_lead_green: phys('#1d8a4b', 0.0, 0.45),
  motor_lead_red: phys('#c7322d', 0.0, 0.45),
  motor_lead_blue: phys('#2a4fd0', 0.0, 0.45),
};
const allMaterials = new Set(Object.values(MATS));

// ---- models -----------------------------------------------------------------------------------------
const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
const load = url => new Promise((res, rej) => loader.load(url, g => res(g.scene), undefined, rej));

function prepare(root, material) {
  root.traverse(o => {
    if (!o.isMesh) return;
    o.castShadow = o.receiveShadow = true;
    if (material) o.material = material;
    else { o.material = o.material.clone(); o.material.clippingPlanes = []; allMaterials.add(o.material); }
  });
  return root;
}

const groups = { motor: new THREE.Group(), magnet: new THREE.Group(), adapter: new THREE.Group(), s1: new THREE.Group(), screws: new THREE.Group() };
Object.values(groups).forEach(g => scene.add(g));

const motorParts = ['front_cap', 'stator', 'rear_cap', 'bearing', 'shaft', 'pilot', 'screws', 'lead_exit',
  'lead_black', 'lead_green', 'lead_red', 'lead_blue'];
await Promise.all([
  ...motorParts.map(async p => groups.motor.add(prepare(await load(`./models/motor_${p}.glb`), MATS[`motor_${p}`]))),
  load('./models/adapter.glb').then(m => groups.adapter.add(prepare(m, MATS.adapter))),
  load('./models/magnet_n.glb').then(m => groups.magnet.add(prepare(m, MATS.magnet_n))),
  load('./models/magnet_s.glb').then(m => groups.magnet.add(prepare(m, MATS.magnet_s))),
  load('./models/plate_screws.glb').then(m => groups.screws.add(prepare(m, MATS.plate_screws))),
  load('./models/s1_screws.glb').then(m => groups.s1.add(prepare(m, MATS.s1_screws))),
  load('./models/tercio-s1.glb').then(m => {
    const s1 = prepare(m);
    const size = new THREE.Box3().setFromObject(s1).getSize(new THREE.Vector3());
    if (size.x < 1) s1.scale.setScalar(1000);            // the board model is in metres
    s1.updateMatrixWorld(true);
    // The plate replaces the S1's NEMA 17 cover: hide it and seat the PCB on the plate's bosses.
    let core = null;
    s1.traverse(o => {
      if (!o.isMesh) return;
      if (/cover/i.test(o.material.name)) o.visible = false;
      if (/^core$/i.test(o.material.name)) core = o;
    });
    const pcb = new THREE.Box3().setFromObject(core ?? s1);
    s1.position.y = layout.z_pcb - pcb.min.y;
    s1.rotation.y = THREE.MathUtils.degToRad(layout.s1_rot);   // turned in the plate (CAD Z = viewer Y)
    groups.s1.add(s1);
  }),
]);
document.getElementById('loading').remove();

// ---- dimensions -------------------------------------------------------------------------------------
const dimLine = new THREE.LineBasicMaterial({ color: 0x3a3f48, transparent: true, opacity: 0.75, depthTest: false });
const dims = new THREE.Group();
dims.renderOrder = 10;
function label(text, pos, parent, cls = '') {
  const el = document.createElement('div');
  el.className = `dim ${cls}`;
  el.textContent = text;
  const o = new CSS2DObject(el);
  o.position.copy(pos);
  parent.add(o);
  return o;
}
function dimension(a, b, off, text, parent) {
  const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b), O = new THREE.Vector3(...off);
  const pts = [A, A.clone().add(O), B.clone().add(O), B];
  const geo = new THREE.BufferGeometry().setFromPoints(pts);
  const l = new THREE.Line(geo, dimLine);
  l.renderOrder = 10;
  parent.add(l);
  label(text, A.clone().add(B).multiplyScalar(0.5).add(O), parent);
}
const h = layout.plate / 2, zt = layout.z_top, [sx, sy] = layout.motor_screws[0];
const adapterDims = new THREE.Group();
// Drawn on the top face so the lines stay on the part instead of floating over the motor.
dimension([-h, zt, h], [h, zt, h], [0, 0, 9], `${layout.plate.toFixed(1)} mm`, adapterDims);
dimension([h, 0, 0], [h, zt, 0], [9, 0, 0], `${layout.thickness.toFixed(2)} mm`, adapterDims);
dimension([-sx, zt + 1.7, sy], [sx, zt + 1.7, sy], [0, 0, 11], `${(2 * sx).toFixed(2)} · 4 × ${layout.motor_screw}`, adapterDims);
dimension([sx, zt + 1.7, sy], [sx, zt + 1.7, -sy], [11, 0, 0], `${(2 * sy).toFixed(1)}`, adapterDims);
groups.adapter.add(adapterDims);
const gapLabel = label(`air gap ${layout.air_gap.toFixed(2)} mm`, new THREE.Vector3(24, layout.magnet_top - 9, 0), groups.magnet);
const gapLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(3.2, layout.magnet_top + 0.5, 0), new THREE.Vector3(16, layout.magnet_top - 9, 0)]), dimLine);
groups.magnet.add(gapLine);
gapLabel.visible = gapLine.visible = false;

// ---- UI ---------------------------------------------------------------------------------------------
const dimsEl = document.getElementById('dims');
[
  ['Plate', `${layout.plate.toFixed(1)} × ${layout.plate.toFixed(1)} × ${layout.thickness.toFixed(2)} mm`],
  ['Outline', 'motor end cap, 0.2 mm inside'],
  ['Motor screws', `4 × ${layout.motor_screw} on ${(2 * layout.motor_screws[0][0]).toFixed(2)} × ${(2 * layout.motor_screws[0][1]).toFixed(1)} mm`],
  ['S1 mounting', '4 × M3 into bosses at the PCB holes'],
  ['PCB underside above rear face', `${layout.z_pcb.toFixed(2)} mm`],
  ['Magnet', `Ø${layout.magnet[0]} × ${layout.magnet[1]} on the flush shaft end`],
  ['AS5600 air gap', `${layout.air_gap.toFixed(2)} mm (spec 0.5–3)`],
  ['Parts clearance to motor', `${layout.parts_clearance.toFixed(2)} mm`],
  ['Mass (6061)', `${layout.mass_g} g`],
].forEach(([k, v]) => { dimsEl.insertAdjacentHTML('beforeend', `<div class="row"><span>${k}</span><span>${v}</span></div>`); });

const state = { explode: 0, section: false, dims: false, spin: false };
const show = { Motor: ['motor'], 'Adapter plate': ['adapter'], Magnet: ['magnet'], 'Tercio S1': ['s1'], 'Plate screws': ['screws'] };
const toggles = document.getElementById('toggles');
function chip(text, pressed, onClick) {
  const b = document.createElement('button');
  b.className = 'chip';
  b.textContent = text;
  b.setAttribute('aria-pressed', String(pressed));
  b.onclick = () => { const v = b.getAttribute('aria-pressed') !== 'true'; b.setAttribute('aria-pressed', String(v)); onClick(v); };
  toggles.appendChild(b);
  return b;
}
for (const [name, keys] of Object.entries(show)) chip(name, true, v => keys.forEach(k => { groups[k].visible = v; }));
chip('Section', false, v => { state.section = v; applySection(); });
chip('Dimensions', false, v => { state.dims = v; showGap(); });
chip('Turntable', false, v => { controls.autoRotate = v; controls.autoRotateSpeed = 1.2; });
// Finish of the adapter.
const FINISHES = { Natural: ['#c8cbd0', 1.0, 0.42], Black: ['#1b1d21', 0.7, 0.45] };
const finishRow = document.createElement('div');
finishRow.className = 'toggles';
toggles.after(finishRow);
const finishChips = [];
for (const [name, [c, met, rough]] of Object.entries(FINISHES)) {
  const b = document.createElement('button');
  b.className = 'chip';
  b.textContent = name === 'Natural' ? 'Bead-blasted aluminium' : 'Black anodised';
  b.setAttribute('aria-pressed', String(name === 'Natural'));
  b.onclick = () => { MATS.adapter.color.set(c); MATS.adapter.metalness = met; MATS.adapter.roughness = rough; finishChips.forEach(x => x.setAttribute('aria-pressed', String(x === b))); };
  finishRow.appendChild(b);
  finishChips.push(b);
}

function showGap() { gapLabel.visible = gapLine.visible = state.section; adapterDims.visible = state.dims && !state.section; }
// Section caps (stencil): every mesh counts its back/front faces behind the plane, and a cap in the
// mesh's colour fills where the count is odd, so cut parts read as solid.
const capGeo = new THREE.PlaneGeometry(600, 600);
const caps = [];
let order = 1;
function addCaps(groupName) {
  const meshes = [];
  groups[groupName].traverse(o => { if (o.isMesh && !o.userData.cap) meshes.push(o); });
  for (const mesh of meshes) {
    const base = { depthWrite: false, depthTest: false, colorWrite: false, stencilWrite: true, stencilFunc: THREE.AlwaysStencilFunc, clippingPlanes: [clip] };
    const back = new THREE.MeshBasicMaterial({ ...base, side: THREE.BackSide });
    back.stencilFail = back.stencilZFail = back.stencilZPass = THREE.IncrementWrapStencilOp;
    const front = new THREE.MeshBasicMaterial({ ...base, side: THREE.FrontSide });
    front.stencilFail = front.stencilZFail = front.stencilZPass = THREE.DecrementWrapStencilOp;
    const m = mesh.material;
    const capMat = new THREE.MeshStandardMaterial({
      color: m.color ?? 0x888888, metalness: Math.min(m.metalness ?? 0.2, 0.5), roughness: Math.max(m.roughness ?? 0.6, 0.55),
      stencilWrite: true, stencilRef: 0, stencilFunc: THREE.NotEqualStencilFunc,
      stencilFail: THREE.ReplaceStencilOp, stencilZFail: THREE.ReplaceStencilOp, stencilZPass: THREE.ReplaceStencilOp,
    });
    const o = order; order += 3;
    const s0 = new THREE.Mesh(mesh.geometry, back), s1 = new THREE.Mesh(mesh.geometry, front);
    for (const x of [s0, s1]) { x.renderOrder = o; x.userData.cap = true; mesh.add(x); }
    const cap = new THREE.Mesh(capGeo, capMat);
    cap.renderOrder = o + 1.1;
    cap.userData.cap = true;
    cap.onAfterRender = r => r.clearStencil();
    groups[groupName].add(cap);
    caps.push(s0, s1, cap);
  }
}
Object.keys(groups).forEach(addCaps);
caps.forEach(c => { c.visible = false; });
showGap();

function applySection() {
  caps.forEach(c => { c.visible = state.section; });
  showGap();
  for (const m of allMaterials) {
    if (m.stencilWrite) continue;
    m.clippingPlanes = state.section ? [clip] : [];
    m.side = state.section ? THREE.DoubleSide : THREE.FrontSide;
    m.needsUpdate = true;
  }
}

const explode = document.getElementById('explode');
explode.oninput = () => {
  state.explode = Number(explode.value);
  document.getElementById('explodeVal').textContent = `${Math.round(state.explode * 100)}%`;
};

const presets = {
  '3/4': [[150, 95, 185], [0, -18, 0]],
  Front: [[0, -20, 240], [0, -20, 0]],
  Side: [[240, -20, 0], [0, -20, 0]],
  Top: [[0.01, 200, 40], [0, 0, 0]],
  Encoder: [[40, 12, 118], [5, 1, 0]],
};
const views = document.getElementById('views');
let tween = null;
for (const [name, [pos, tgt]] of Object.entries(presets)) {
  const b = document.createElement('button');
  b.className = 'chip';
  b.textContent = name;
  b.onclick = () => {
    tween = { t: 0, p0: camera.position.clone(), t0: controls.target.clone(), p1: new THREE.Vector3(...pos), t1: new THREE.Vector3(...tgt) };
    // The encoder close-up is only useful cut open.
    if (name === 'Encoder' && !state.section) [...document.querySelectorAll('.chip')].find(c => c.textContent === 'Section')?.click();
  };
  views.appendChild(b);
}
camera.position.set(...presets['3/4'][0]);

// ---- loop -------------------------------------------------------------------------------------------
const ease = x => (x < 0.5 ? 4 * x ** 3 : 1 - (-2 * x + 2) ** 3 / 2);
let shown = 0, framed = 0;
function lensShift() {
  const shift = innerWidth > 760 ? -Math.min(200, innerWidth * 0.12) : 0;
  camera.setViewOffset(innerWidth, innerHeight, shift, 0, innerWidth, innerHeight);
}
lensShift();
renderer.setAnimationLoop(() => {
  shown += (state.explode - shown) * 0.18;
  // Keep the exploded stack in frame: lift the target and pull back as it opens.
  const want = shown;
  const d = want - framed;
  if (Math.abs(d) > 1e-4) {
    const off = camera.position.clone().sub(controls.target);
    const k = (1 + 0.3 * want) / (1 + 0.3 * framed);
    controls.target.y += 16 * d;
    camera.position.copy(controls.target).add(off.multiplyScalar(k));
    framed = want;
  }
  groups.magnet.position.y = 8 * shown;
  groups.adapter.position.y = 26 * shown;
  groups.s1.position.y = 48 * shown;
  groups.screws.position.y = 44 * shown;
  if (tween) {
    tween.t = Math.min(1, tween.t + 0.03);
    const k = ease(tween.t);
    camera.position.lerpVectors(tween.p0, tween.p1, k);
    controls.target.lerpVectors(tween.t0, tween.t1, k);
    if (tween.t >= 1) tween = null;
  }
  controls.update();
  renderer.render(scene, camera);
  labels.render(scene, camera);
});

window.viewer = { camera, controls, setExplode: v => { explode.value = v; explode.oninput(); }, click: t => [...document.querySelectorAll('.chip')].find(b => b.textContent === t)?.click() };
addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  lensShift();
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
  labels.setSize(innerWidth, innerHeight);
});
