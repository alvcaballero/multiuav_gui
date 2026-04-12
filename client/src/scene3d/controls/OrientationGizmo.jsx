import { useRef, useMemo, useEffect } from 'react';
import { useThree, useFrame } from '@react-three/fiber';
import * as THREE from 'three';

// ── constants ──────────────────────────────────────────────────────────────────
const GIZMO_SIZE = 120; // px — square viewport in corner
const GIZMO_CAMERA_DIST = 3.5;
const ANIM_DURATION = 0.4; // seconds

// BoxGeometry material index order: +X, -X, +Y, -Y, +Z, -Z
const BOX_FACE_ORDER = [
  { label: 'E', normal: new THREE.Vector3( 1,  0,  0), color: '#3498db' },
  { label: 'W', normal: new THREE.Vector3(-1,  0,  0), color: '#2980b9' },
  { label: 'T', normal: new THREE.Vector3( 0,  1,  0), color: '#2ecc71' },
  { label: 'B', normal: new THREE.Vector3( 0, -1,  0), color: '#27ae60' },
  { label: 'S', normal: new THREE.Vector3( 0,  0,  1), color: '#e74c3c' },
  { label: 'N', normal: new THREE.Vector3( 0,  0, -1), color: '#c0392b' },
];

// ── label texture ──────────────────────────────────────────────────────────────
function makeLabelTexture(label, bgColor) {
  const size = 128;
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = bgColor;
  ctx.fillRect(0, 0, size, size);
  ctx.strokeStyle = 'rgba(0,0,0,0.3)';
  ctx.lineWidth = 6;
  ctx.strokeRect(3, 3, size - 6, size - 6);
  ctx.fillStyle = '#ffffff';
  ctx.font = `bold ${Math.round(size * 0.44)}px sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, size / 2, size / 2);
  return new THREE.CanvasTexture(canvas);
}

// ── component ──────────────────────────────────────────────────────────────────
export default function OrientationGizmo({ controlsRef }) {
  const { gl, camera: mainCamera, scene: mainScene, size } = useThree(); // mainScene needed for manual render

  // ── isolated scene + camera for the gizmo ───────────────────────────────────
  const { gizmoScene, gizmoBox } = useMemo(() => {
    const gizmoScene = new THREE.Scene();
    gizmoScene.add(new THREE.AmbientLight(0xffffff, 0.8));
    const dir = new THREE.DirectionalLight(0xffffff, 1.2);
    dir.position.set(5, 8, 5);
    gizmoScene.add(dir);

    const materials = BOX_FACE_ORDER.map(
      (f) => new THREE.MeshStandardMaterial({ map: makeLabelTexture(f.label, f.color) })
    );
    const gizmoBox = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), materials);
    gizmoScene.add(gizmoBox);

    return { gizmoScene, gizmoBox };
  }, []);

  const gizmoCam = useMemo(() => new THREE.PerspectiveCamera(50, 1, 0.1, 100), []);

  // ── animation state ──────────────────────────────────────────────────────────
  const animRef = useRef(null);

  const startAnimRef = useRef(null);
  startAnimRef.current = (faceNormal) => {
    if (!controlsRef.current) return;
    const controls = controlsRef.current;
    const target = controls.target.clone();
    const dist = mainCamera.position.clone().sub(target).length();

    let toPos = target.clone().addScaledVector(faceNormal, dist);
    if (Math.abs(faceNormal.y) > 0.9) {
      toPos = target.clone().add(new THREE.Vector3(0.001, dist * faceNormal.y, 0.001));
    }

    const fromPos = mainCamera.position.clone();
    const fromQ   = mainCamera.quaternion.clone();

    const tempCam = mainCamera.clone();
    tempCam.position.copy(toPos);
    tempCam.lookAt(target);
    const toQ = tempCam.quaternion.clone();

    animRef.current = { fromPos, toPos, fromQ, toQ, elapsed: 0 };
  };

  // ── click handler ────────────────────────────────────────────────────────────
  useEffect(() => {
    const canvas = gl.domElement;
    const raycaster = new THREE.Raycaster();

    const onClick = (e) => {
      const rect = canvas.getBoundingClientRect();
      const px = e.clientX - rect.left;
      const py = e.clientY - rect.top;

      const vx = size.width  - GIZMO_SIZE;
      const vy = size.height - GIZMO_SIZE;
      if (px < vx || py < vy) return;

      const ndcX =  ((px - vx) / GIZMO_SIZE) * 2 - 1;
      const ndcY = -((py - vy) / GIZMO_SIZE) * 2 + 1;

      raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), gizmoCam);
      const hits = raycaster.intersectObject(gizmoBox);
      if (!hits.length) return;

      const faceIndex = Math.floor(hits[0].faceIndex / 2);
      startAnimRef.current(BOX_FACE_ORDER[faceIndex].normal);
    };

    canvas.addEventListener('click', onClick);
    return () => canvas.removeEventListener('click', onClick);
  }, [gl, size, gizmoCam, gizmoBox]);

  // ── per-frame ────────────────────────────────────────────────────────────────
  // Priority 1: R3F skips its own render when any useFrame has priority > 0,
  // so we MUST render the main scene ourselves first, then draw the gizmo on top.
  useFrame((_, delta) => {
    // 1. Animate main camera
    const anim = animRef.current;
    if (anim && controlsRef.current) {
      anim.elapsed += delta;
      const t = Math.min(anim.elapsed / ANIM_DURATION, 1);
      const ease = t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;

      mainCamera.position.lerpVectors(anim.fromPos, anim.toPos, ease);
      mainCamera.quaternion.slerpQuaternions(anim.fromQ, anim.toQ, ease);
      controlsRef.current.update();

      if (t >= 1) animRef.current = null;
    }

    const w   = size.width;
    const h   = size.height;
    const s   = GIZMO_SIZE;
    const dpr = gl.getPixelRatio();

    // 2. Render main scene (full viewport)
    gl.autoClear = true;
    gl.setViewport(0, 0, Math.round(w * dpr), Math.round(h * dpr));
    gl.setScissorTest(false);
    gl.render(mainScene, mainCamera);

    // 3. Sync gizmo box rotation
    gizmoBox.quaternion.copy(mainCamera.quaternion).invert();
    gizmoCam.position.set(0, 0, GIZMO_CAMERA_DIST);

    // 4. Render gizmo on top — bottom-right corner
    const vx = Math.round((w - s) * dpr);
    const vw = Math.round(s * dpr);
    const vh = Math.round(s * dpr);

    gl.autoClear = false;
    gl.setScissorTest(true);
    gl.setScissor(vx, 0, vw, vh);
    gl.setViewport(vx, 0, vw, vh);
    gl.clearDepth();
    gl.render(gizmoScene, gizmoCam);

    // Restore
    gl.setScissorTest(false);
    gl.setViewport(0, 0, Math.round(w * dpr), Math.round(h * dpr));
    gl.autoClear = true;
  }, 1);

  return null;
}
