# React Doctor - False Positives

## react-doctor/rerender-state-only-in-handlers

- `src/SocketController.jsx` `notifications` - leído dentro de `useEffect([notifications])` que dispara `enqueueSnackbar` (toast visible en pantalla); convertir a `useRef` rompería el disparo del efecto porque mutar `.current` no re-ejecuta efectos.
- `src/scene3d/primitives/NumberedSphere.jsx` `hovered` - leído dentro de dos `useEffect([hovered])` que mutan el cursor del mouse y el color del material 3D; convertir a `useRef` rompería el feedback visual de hover.

## react-doctor/no-array-index-as-key

- `src/components/mission/MissionElevation.jsx:208` `index` - el índice del `.map()` sobre `elevProfile` ES el identificador de negocio real (número de ruta), usado como `value={index}` y como key de lookup en `elevProfile[selectRT]` en el resto del componente; no es un accidente de posición de array.
- `src/pages/LinearGauge.jsx:89` `i` - la variable se llama `i` pero es un valor de tick matemático calculado en un `for (let i = start; i <= end; i += tickInterval)`, no una posición de array; ya es único por construcción.

## react-doctor/no-unknown-property

- Todo `src/scene3d/**/*.jsx` (65 casos en 9 archivos: `R3FCanvas.jsx`, `Drone.jsx`, `R3FDevices.jsx`, `MapTileGround.jsx`, `Arrow3D.jsx`, `NumberedSphere.jsx`, `MapVectorGround.jsx`, `Polyhedron.jsx`, `R3DMarkers.jsx`) - elementos JSX en minúscula como `<mesh>`, `<group>`, `<meshStandardMaterial>`, `<perspectiveCamera>`, `<primitive>` son el reconciler custom de React Three Fiber (renderiza a objetos Three.js, no al DOM), no elementos HTML. Props como `position`, `rotation`, `args`, `attach`, `intensity`, `map`, `roughness`, `metalness`, `depthTest`/`depthWrite`, `sunPosition`, `rayleigh`, `turbidity` son válidas en R3F y no tienen equivalente HTML. El linter estático de `oxlint-builtin:react` no reconoce el reconciler de R3F y trata estos elementos como si fueran DOM plano. Verificado archivo por archivo: no hay ningún elemento HTML real ni typo genuino (class/for/tabindex) mezclado en estos 9 archivos - son 100% código de escena 3D.

## react-doctor/no-json-parse-stringify-clone

- `src/store/mission.js:217` (`copyWaypoint` reducer) - `state.route[routeIndex].wp[wpIndex]` dentro de un reducer de Redux Toolkit (`createSlice`) es un draft/Proxy de Immer, no un objeto plano. Verificado con un test real: `structuredClone()` sobre un draft de Immer lanza `DOMException: could not be cloned` porque el algoritmo de clonado estructurado no puede serializar el Proxy. `JSON.parse(JSON.stringify(x))` funciona acá porque pasa por texto plano y escapa el proxy; `structuredClone` no tiene ese efecto secundario y por eso falla. No convertir sin antes envolver en algo que materialice el draft a un objeto plano primero (ej. `current(state.route[...])` de Immer).
