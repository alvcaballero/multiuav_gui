# React Doctor - False Positives

## react-doctor/rerender-state-only-in-handlers

- `src/SocketController.jsx` `notifications` - leído dentro de `useEffect([notifications])` que dispara `enqueueSnackbar` (toast visible en pantalla); convertir a `useRef` rompería el disparo del efecto porque mutar `.current` no re-ejecuta efectos.
- `src/scene3d/primitives/NumberedSphere.jsx` `hovered` - leído dentro de dos `useEffect([hovered])` que mutan el cursor del mouse y el color del material 3D; convertir a `useRef` rompería el feedback visual de hover.
