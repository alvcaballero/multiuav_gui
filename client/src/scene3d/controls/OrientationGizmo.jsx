import { useRef, useMemo, useEffect } from 'react';
import { useThree, useFrame } from '@react-three/fiber';
import * as THREE from 'three';

// ── constants ──────────────────────────────────────────────────────────────────
const GIZMO_SIZE = 120; // px — square viewport in corner
const GIZMO_CAMERA_DIST = 3.5;
const ANIM_DURATION = 0.4; // seconds

// BoxGeometry material index order: +X, -X, +Y, -Y, +Z, -Z
const BOX_FACE_ORDER = [
  { label: 'E', normal: new THREE.Vector3(1, 0, 0), color: '#3498db' },
  { label: 'W', normal: new THREE.Vector3(-1, 0, 0), color: '#2980b9' },
  { label: 'T', normal: new THREE.Vector3(0, 1, 0), color: '#2ecc71' },
  { label: 'B', normal: new THREE.Vector3(0, -1, 0), color: '#27ae60' },
  { label: 'S', normal: new THREE.Vector3(0, 0, 1), color: '#e74c3c' },
  { label: 'N', normal: new THREE.Vector3(0, 0, -1), color: '#c0392b' },
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
  const { gl } = useThree();

  // ── isolated scene + camera for the gizmo ───────────────────────────────────
  const { gizmoScene, gizmoBox } = useMemo(() => {
    const gizmoScene = new THREE.Scene();
    gizmoScene.add(new THREE.AmbientLight(0xffffff, 0.8));
    const dir = new THREE.DirectionalLight(0xffffff, 1.2);
    dir.position.set(5, 8, 5);
    gizmoScene.add(dir);

    const materials = BOX_FACE_ORDER.map(
      (f) => new THREE.MeshStandardMaterial({ map: makeLabelTexture(f.label, f.color) }),
    );
    const gizmoBox = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), materials);
    gizmoScene.add(gizmoBox);

    return { gizmoScene, gizmoBox };
  }, []);

  const gizmoCam = useMemo(() => new THREE.PerspectiveCamera(50, 1, 0.1, 100), []);

  // ── animation state ──────────────────────────────────────────────────────────
  const animRef = useRef(null);

  const cameraRef = useRef(null);
  const startAnimRef = useRef(null);
  startAnimRef.current = (faceNormal) => {
    if (!controlsRef.current || !cameraRef.current) return;
    const cam = cameraRef.current;
    const controls = controlsRef.current;
    const target = controls.target.clone();
    const dist = cam.position.clone().sub(target).length();

    let toPos = target.clone().addScaledVector(faceNormal, dist);
    if (Math.abs(faceNormal.y) > 0.9) {
      toPos = target.clone().add(new THREE.Vector3(0.001, dist * faceNormal.y, 0.001));
    }

    const fromPos = cam.position.clone();
    const fromQ = cam.quaternion.clone();

    const tempCam = cam.clone();
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

      const w = rect.width;
      const h = rect.height;
      const vx = w - GIZMO_SIZE;
      const vy = h - GIZMO_SIZE;
      if (px < vx || py < vy) return;

      const ndcX = ((px - vx) / GIZMO_SIZE) * 2 - 1;
      const ndcY = -((py - vy) / GIZMO_SIZE) * 2 + 1;

      raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), gizmoCam);
      const hits = raycaster.intersectObject(gizmoBox);
      if (!hits.length) return;

      const faceIndex = Math.floor(hits[0].faceIndex / 2);
      startAnimRef.current(BOX_FACE_ORDER[faceIndex].normal);
    };

    canvas.addEventListener('click', onClick);
    return () => canvas.removeEventListener('click', onClick);
  }, [gl, gizmoCam, gizmoBox]);

  // ── per-frame ────────────────────────────────────────────────────────────────
  // Priority 1: R3F skips its own render when any useFrame has priority > 0,
  // so we MUST render the main scene ourselves first, then draw the gizmo on top.
  useFrame(({ scene, camera, size: frameSize, gl: frameGl }, delta) => {
    cameraRef.current = camera;
    // 1. Animate main camera
    const anim = animRef.current;
    if (anim && controlsRef.current) {
      anim.elapsed += delta;
      const t = Math.min(anim.elapsed / ANIM_DURATION, 1);
      const ease = t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;

      camera.position.lerpVectors(anim.fromPos, anim.toPos, ease);
      camera.quaternion.slerpQuaternions(anim.fromQ, anim.toQ, ease);
      controlsRef.current.update();

      if (t >= 1) animRef.current = null;
    }

    const w = frameSize.width;
    const h = frameSize.height;
    const s = GIZMO_SIZE;

    // 2. Render main scene (full viewport) — THREE multiplies by dpr internally
    frameGl.autoClear = true;
    frameGl.setViewport(0, 0, w, h);
    frameGl.setScissorTest(false);
    frameGl.render(scene, camera);

    // 3. Sync gizmo box rotation
    gizmoBox.quaternion.copy(camera.quaternion).invert();
    gizmoCam.position.set(0, 0, GIZMO_CAMERA_DIST);

    // 4. Render gizmo on top — bottom-right corner (WebGL Y=0 is bottom)
    frameGl.autoClear = false;
    frameGl.setScissorTest(true);
    frameGl.setScissor(w - s, 0, s, s);
    frameGl.setViewport(w - s, 0, s, s);
    frameGl.clearDepth();
    frameGl.render(gizmoScene, gizmoCam);

    // Restore
    frameGl.setScissorTest(false);
    frameGl.setViewport(0, 0, w, h);
    frameGl.autoClear = true;
  }, 1);

  return null;
}
