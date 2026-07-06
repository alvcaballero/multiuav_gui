# React Doctor - False Positives

## react-doctor/rerender-state-only-in-handlers

- `src/SocketController.jsx` `notifications` - leído dentro de `useEffect([notifications])` que dispara `enqueueSnackbar` (toast visible en pantalla); convertir a `useRef` rompería el disparo del efecto porque mutar `.current` no re-ejecuta efectos.
- `src/scene3d/primitives/NumberedSphere.jsx` `hovered` - leído dentro de dos `useEffect([hovered])` que mutan el cursor del mouse y el color del material 3D; convertir a `useRef` rompería el feedback visual de hover.

## react-doctor/no-array-index-as-key

- `src/components/mission/MissionElevation.jsx:208` `index` - el índice del `.map()` sobre `elevProfile` ES el identificador de negocio real (número de ruta), usado como `value={index}` y como key de lookup en `elevProfile[selectRT]` en el resto del componente; no es un accidente de posición de array.
- `src/pages/LinearGauge.jsx:89` `i` - la variable se llama `i` pero es un valor de tick matemático calculado en un `for (let i = start; i <= end; i += tickInterval)`, no una posición de array; ya es único por construcción.
