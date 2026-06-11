// Embedded 3D twin for the Telamoto web UI.
//
// Renders the robot CLIENT-SIDE (three.js + urdf-loader) from the URDF served
// at /api/urdf, driven by the ~25 Hz binary state stream on /ws3d:
//   0x01 . 6×f32 joint positions (UR order) . box count . 10×f32 per box
//   (xyz, quat xyzw, size) — little-endian, see _twin_frame() server-side.
// A few kB/s total, so the view stays live on the same laggy links the jog
// compensation targets. The planning-scene wall renders as a translucent box;
// the TCP path is drawn as a trail from client-side FK (tool0 world position).

import * as THREE from 'three';
import { OrbitControls } from '/static/OrbitControls.js';
import URDFLoader from '/static/URDFLoader.js';

const UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'];
const TRAIL_MAX = 600;            // points (~24 s of motion at 25 Hz)
const TRAIL_MIN_STEP = 0.002;     // m — don't grow the trail at standstill

export function initTwin(container) {
  function fail(msg) {
    const note = document.createElement('div');
    note.style.cssText = 'padding:1rem;color:#e3b341;font-size:.85rem';
    note.textContent = msg;
    container.style.height = 'auto';
    container.appendChild(note);
    return null;
  }
  // Preflight WebGL: Chrome 131+ refuses SOFTWARE WebGL outright, so on a host
  // without GPU acceleration (e.g. no GL driver on a realtime kernel) the
  // renderer constructor throws — probe first and explain instead of leaving a
  // silent black box.
  const probe = document.createElement('canvas');
  if (!(probe.getContext('webgl2') || probe.getContext('webgl')))
    return fail('WebGL is not available in this browser. On a PC without GPU '
      + 'acceleration Chrome blocks software WebGL — use Firefox on this PC, '
      + 'open the UI from another device (phone/laptop), or start Chrome with '
      + '--enable-unsafe-swiftshader.');

  THREE.Object3D.DEFAULT_UP.set(0, 0, 1);          // ROS is Z-up

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x181818);
  scene.add(new THREE.HemisphereLight(0xbbbbbb, 0x333344, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.2);
  sun.position.set(1.5, -1, 2.5);
  scene.add(sun);
  const grid = new THREE.GridHelper(3, 12, 0x4a4a4a, 0x2e2e2e);
  grid.rotation.x = Math.PI / 2;                   // GridHelper is XZ; ground is XY
  scene.add(grid);
  scene.add(new THREE.AxesHelper(0.25));

  const camera = new THREE.PerspectiveCamera(50, 1, 0.05, 30);
  camera.position.set(1.8, -1.4, 1.3);
  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true });
  } catch (e) {
    // The probe context can succeed while three's (webgl2, antialiased) one is
    // refused — same root cause, same advice.
    return fail('WebGL init failed (' + e.message + '). On a PC without GPU '
      + 'acceleration Chrome blocks software WebGL — use Firefox on this PC, '
      + 'open the UI from another device (phone/laptop), or fix the GPU driver.');
  }
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  container.appendChild(renderer.domElement);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.5);
  controls.enableDamping = true;

  const overlay = document.createElement('div');
  overlay.style.cssText = 'position:relative;top:-1.6rem;height:0;text-align:center;' +
    'color:#888;font-size:.8rem;pointer-events:none';
  container.appendChild(overlay);

  function resize() {
    const w = container.clientWidth, h = container.clientHeight || 340;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }
  new ResizeObserver(resize).observe(container);
  resize();

  // ── robot ──────────────────────────────────────────────────────────────
  let robot = null, tool = null;
  async function loadRobot() {
    try {
      const r = await fetch('/api/urdf');
      if (!r.ok) throw new Error('urdf ' + r.status);
      const loader = new URDFLoader();
      loader.packages = pkg => '/pkg/' + pkg;       // package:// → our mesh server
      robot = loader.parse(await r.text());
      scene.add(robot);
      tool = robot.links['tool0'] || null;
      overlay.textContent = '';
    } catch (e) {
      overlay.textContent = 'waiting for robot_description…';
      setTimeout(loadRobot, 2000);                  // bringup race → retry
    }
  }
  loadRobot();

  // ── planning-scene boxes (the draggable test wall) ─────────────────────
  // Small obstacles stay clearly visible; large cage walls are made faint —
  // the camera usually looks at the robot THROUGH 2-3 of them, and stacked
  // 0.45-opacity panels wash the whole scene out.
  const boxMat = new THREE.MeshStandardMaterial(
    { color: 0xc4a24a, transparent: true, opacity: 0.45, depthWrite: false });
  const cageMat = new THREE.MeshStandardMaterial(
    { color: 0xc4a24a, transparent: true, opacity: 0.1, depthWrite: false });
  const boxGeo = new THREE.BoxGeometry(1, 1, 1);
  const boxes = [];                                 // pooled meshes, count = frame N
  let wallsVisible = true;                          // view-only toggle (web UI button):
  let lastN = 0;                                    // collision checking is unaffected
  function setBoxes(view, off, n) {
    lastN = n;
    while (boxes.length < n) {
      const m = new THREE.Mesh(boxGeo, boxMat);
      scene.add(m);
      boxes.push(m);
    }
    boxes.forEach((m, i) => { m.visible = wallsVisible && i < n; });
    for (let i = 0; i < n; i++, off += 40) {
      const m = boxes[i];
      m.position.set(view.getFloat32(off, true), view.getFloat32(off + 4, true),
                     view.getFloat32(off + 8, true));
      m.quaternion.set(view.getFloat32(off + 12, true), view.getFloat32(off + 16, true),
                       view.getFloat32(off + 20, true), view.getFloat32(off + 24, true));
      m.scale.set(view.getFloat32(off + 28, true) || 1e-4,
                  view.getFloat32(off + 32, true) || 1e-4,
                  view.getFloat32(off + 36, true) || 1e-4);
      m.material = Math.max(m.scale.x, m.scale.y, m.scale.z) > 1.5 ? cageMat : boxMat;
    }
  }

  // ── TCP trail (client-side FK: tool0 world position after joint update) ─
  const trailPos = new Float32Array(TRAIL_MAX * 3);
  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
  trailGeo.setDrawRange(0, 0);
  scene.add(new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0x4ea1ff })));
  let trailN = 0;
  const tcp = new THREE.Vector3(), lastTcp = new THREE.Vector3(1e9, 0, 0);
  function pushTrail() {
    if (!tool) return;
    tool.getWorldPosition(tcp);
    if (tcp.distanceTo(lastTcp) < TRAIL_MIN_STEP) return;
    lastTcp.copy(tcp);
    if (trailN === TRAIL_MAX) {                     // full → drop the oldest point
      trailPos.copyWithin(0, 3);
      trailN--;
    }
    trailPos.set([tcp.x, tcp.y, tcp.z], trailN++ * 3);
    trailGeo.attributes.position.needsUpdate = true;
    trailGeo.setDrawRange(0, trailN);
  }

  // ── state stream ───────────────────────────────────────────────────────
  let ws = null, closed = false;
  function wsOpen() {
    if (closed) return;
    ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://')
                       + location.host + '/ws3d');
    ws.binaryType = 'arraybuffer';
    ws.onmessage = ev => {
      const v = new DataView(ev.data);
      if (v.getUint8(0) !== 1) return;
      if (robot) {
        for (let i = 0; i < 6; i++)
          robot.setJointValue(UR_JOINTS[i], v.getFloat32(1 + 4 * i, true));
        robot.updateMatrixWorld(true);
        pushTrail();
      }
      setBoxes(v, 26, v.getUint8(25));
    };
    ws.onclose = () => { ws = null; if (!closed) setTimeout(wsOpen, 1000); };
  }
  wsOpen();

  // ── render loop / lifecycle ────────────────────────────────────────────
  let raf = 0;
  function loop() {
    raf = requestAnimationFrame(loop);
    controls.update();
    renderer.render(scene, camera);
  }
  loop();

  return {
    pause() {                                       // hidden: stop GPU + bandwidth
      closed = true;
      cancelAnimationFrame(raf);
      if (ws) ws.close();
    },
    resume() {
      closed = false;
      wsOpen();
      loop();
    },
    walls(show) {                                   // hide/show the cage rendering
      wallsVisible = show;
      boxes.forEach((m, i) => { m.visible = show && i < lastN; });
    },
  };
}
