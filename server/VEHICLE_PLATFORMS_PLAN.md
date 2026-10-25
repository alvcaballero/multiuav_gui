# Device models, tipos de vehículo y capacidades — plan

Objetivo: que la flota pueda ser heterogénea (multirrotor, ala fija, VTOL, USV, UGV) y que cada device diga QUÉ
es (tipo, física, capacidades) por separado de CÓMO se le habla (protocolo, stack, mensajes). Antes las dos
cosas estaban fundidas en la "categoría", y eso producía una categoría por combinación.

## Cómo retomar (estado al 2026-10-25, cierre de sesión)

**Leer primero:** esta sección y "Pendiente". El código está documentado donde vive, no acá:
- `server/CLAUDE.md` › ROS Integration: interfaces, **Device models**, **Capabilities**, **Mission content follows
  the vehicle** (incluye la geometría en ala); › Mission Planning System (formato v4, `altitude_ref`); › MAVLink
  Protocol › Missions (VTOL, altura relativa, `test:e2e:vtol`).
- `client/CLAUDE.md` › `components/devices/` y `mission/`, Missions as task graphs, Vehicle state (`vtolState`).
- Lo suelto que no es de este plan: `server/TODO.md`.

**El modelo mental en una línea:** device = **modelo** (QUÉ es y qué PUEDE: tipo, firmware, capacidades, física)
\+ **interfaz** (`category`: CÓMO se le habla) + **protocolo** (por dónde viaja) + atributos del device
(systemId, slow, overrides de capacidades). **Lo que una misión puede llevar = lo que la interfaz codifica ∩ lo
que el vehículo puede** (capacidades efectivas + `vehicleType`).

**Hecho** (rama `feat/eve-agent-integration`, sin push):

| Fase | Commits | Qué quedó |
|---|---|---|
| F1.0 / F1 | `0506110`… | Un archivo por interfaz, validador único, `extends`/`abstract`, `firmwares`, `vehicleTypes`, `mission:`, `defaultModel` |
| F2 | …`e615a79` | Device models (`config/devices/device_models/`), `Device.deviceModel`, formulario nombre → tipo → modelo → interfaz → protocolo |
| F3 | …`e615a79`, `eef1e5a` | Capacidades (`capabilities.yaml`, standard/optional/`'*'`, overrides); lista de devices con indicador RTK y filtro por tipo de vehículo |
| fix altura | `ab78624` | `altitude_ref` por tarea: relativa por defecto → `MAV_FRAME_GLOBAL_RELATIVE_ALT` |
| F4/F5 VTOL | `7435b0d`, `b54f9ff` | Eje `vtolState`; acción `vtol_transition`, `NAV_VTOL_LAND`; **volado en SITL** (`npm run test:e2e:vtol`) |
| F5 contenido | `7c2f0e6` | Capacidades dueñas de items de misión (`camera` nueva), `vehicleTypes` en `mission_schema.yaml`, `missionValidator.js` (rechaza en los 3 caminos de carga), `POST /missions/check` + botón Validar, endpoints con `?deviceId=`, formulario del device según capacidad |
| F5 geometría | `04d82ce` | Warnings de geometría en ala (transición, giros, frenado, altura) con los números del modelo; `standard_vtol` con valores de PX4 SITL |

**Cómo probar:**
- `npm test` SIEMPRE (DB en memoria, `LOG_FILES=false`: no toca `server/logs/`). Falla `test-definition-resolver.js`
  (windTurbine): es anterior a todo esto. `test-raw-tap.js:161` (`ageMs >= 20` tras `sleep(20)`) falla a veces con la
  suite en paralelo y pasa solo: el timer dispara 1–2 ms antes de lo que mide `Date.now()`.
- Depurar un vuelo: los logs del server rotan con nombres fijos, el activo es siempre `server/logs/<nombre>.log`
  (`mission.log`, `combined.log`…) y los anteriores `<nombre>1..N`.
- El VTOL: SITL (`make px4_sitl gz_standard_vtol`, en tierra), server en `:4000` con `MAVLINK_CONNECTION=true`,
  un device `standard_vtol` / `px4_mavlink` / mavlink con su `systemId`; `npm run test:e2e:vtol` vuela la misión,
  la sigue desde el GCS y desde PX4 (`px4-listener`), manda ReturnHome/Land si falla y guarda el ULog.
- El server corre con `node` sin watch: después de un cambio hay que reiniciarlo (perdimos un intento por eso).

**Trampas del proyecto** (no repetirlas):
- `Map.groupBy`/`Object.groupBy` no existen en Node 20 (el server).
- `SelectField` mantiene su lista vieja hasta que responde un endpoint nuevo: remontarlo con `key={endpoint}`.
- `fd` ignora lo que está en `.gitignore` (el `build/` de PX4): usar `fd -u`. En `rg`, `-h` es la ayuda (sin nombre
  de archivo es `-I`).
- Separar un commit por hunks con `git apply --unidiff-zero` UBICA MAL las líneas (salió un commit roto): aplicar
  los hunks por número de línea ORIGINAL sobre `HEAD` y verificar cada commit aislado en un worktree.
- Otra sesión commitea en este repo en paralelo: mirar `git log` antes de commitear y no usar `git stash`.
- Comportamiento de PX4: leerlo en `~/PX4-Autopilot` (navigator, `FeasibilityChecker.cpp`, ROMFS), no de memoria.

## Pendiente

**Del usuario (fuera del código):**
- Pasar `vtol_1` a `standard_vtol` (quedó como `x500` en la migración de F2) y cambiar el launch del relay del dron a
  `/<device>/slow/<topic completo>`.
- Habilitar `rtk` en los M210/M300 que tengan el módulo, y `gimbal` / `camera` / `camera_stream` / `lidar_3d` en los
  x500 que los tengan (son opcionales: apagados hasta habilitarlos).
- Verificar los datos físicos de los DJI (copias sin verificar del viejo `devices.yaml`) y la interfaz `sjtu`.

**Chicos (código), en este orden:**
1. **No armar sobre una misión que PX4 marcó inválida.** Hoy el GCS arma, PX4 rechaza el modo (`TEMPORARILY_REJECTED`)
   y desarma solo. El dato existe (`mission_result.valid` por MAVLink).
2. **Mostrar los warnings del validador al cargar**, no solo con el botón Validar.
3. Validar mientras se edita (con debounce) y marcar el waypoint con error dentro del editor (hoy: lista + click).
4. Que el server rechace `camera[]` / `files[]` de un device sin `camera_stream` / `file_download` (hoy solo se ocultan
   en el formulario, sin borrarlos).
5. Leer los parámetros del vehículo por MAVLink (`PARAM_REQUEST_READ`) para verificar el modelo contra el vehículo
   real; con eso, la geometría en ala podría pasar de warning a error.
6. Sin verificar: `stay` (hold) en FW; en qué referencia de altura manda el planner externo de `planning.js`
   (`/get_plan`; sin marca se toma relativa); el total de tests varía entre corridas (941–976) sin fallos nuevos
   (el único inestable conocido es el de `test-raw-tap.js`, ver "Cómo probar").

**En curso — F6 · Planificación y seguridad.** Los planners reciben el modelo y planifican con los mismos números
que valida el GCS. Estado de partida (2026-10-25): TODOS recibían solo `category` y la usaban como modelo.
- **F6.1 · GCS (hecho, `c967c9a`; lidar a bordo `4acb9ca`):** `planningProfileOf(device)` → `vehicle` en el request a Smart-Flies y en
  `geodetic-to-xyz`; la identidad en `geodetic-to-xyz` es nombre + id (ya no `category`); los tipos de misión de
  `config/planning/config.yaml` piden `vehicleTypes` + `capabilities` (`planningRequirements.js`), no categorías.
- **F6.2 · Camino LLM (hecho: mcp_server `87875e2` en la rama `feat/vehicle-aware-planning`, GCS `d1ae2cb`):** `get_devices` muestra `Vehicle` (tipo +
  modelo) y `Capabilities`; `default.md`/`defaultFast.md` eligen solo multirrotor/VTOL con `autonomous_mission` +
  `camera` (si ninguno califica, frenan y dicen qué falta); velocidad: `cruise_speed: vehicle` por defecto → el
  planner pone en cada ruta el `vehicle.speeds.mission` de su dron (5 si no lo declara), un número del usuario vale
  para todos (`planner.md` y `plannerFast.md`, misma línea); el planner LLM recibe `vehicle` recortado (sin
  `physics`/`battery`/sensores ni `category`, `plannerDeviceView` en el MCP); `selected_devices` ya no lleva
  `category`; `uav_type` opcional (lo llena el GCS); ejemplo de `verification-mission.md` sin `uav_type` ni
  `mode_gimbal` (era el pendiente 5). El planner LLM NO emite transiciones VTOL: un VTOL vuela todo en
  multicóptero.
- **F6.2b · eve (hecho, sin commit; `gcs_eevee_assistant` + `get_devices` del MCP):** la skill `mission-planning.md`
  con las mismas reglas de selección y velocidad; `request_mission_plan.ts` sin `category` y con la misma vista
  recortada del device que el MCP (copia profunda por JSON: toon encodea `null` un objeto de otro realm); el planner
  LLM (`subagents/planner`) con la línea de velocidad de `planner.md`; los DOS sandbox resuelven `cruise_speed:
  vehicle` por dron (`cruise_speed()` en su `build_mission.py`, desde `vehicle.speeds.mission` de `devices.json`) y
  ya no piden `category`; el nuevo (`agents/planner_sandbox`) no emite `uav_type` (el GCS lo pisa al cargar,
  `commandRegistry.js`). `get_devices` ya no muestra `Category`.
  Visto y NO tocado: el asistente delega en `subagents/planner_sandbox` (el viejo, `route[]`); el que emite
  `tasks[]` (`agents/planner_sandbox`) es un agente de prueba. Y su `DEFAULT_PARAMS` lleva `mode_gimbal: 0`, que el
  validador del GCS rechaza en un dron sin `gimbal` (un x500 sin habilitarlo).
- **F6.3 · Smart-Flies** (`~/work/px4/Smart-Flies-MUAVPlanner`, la que lanza `run.yml`): usar `vehicle` si viene,
  su `files/devices.yaml` si no (hoy busca por `category` y cae en el M300 SIN avisar); la rama de ala fija por
  `vehicleType` (hoy `category == "px4"`, que no existe: muerta); emitir `vtol_transition` en vez de `{command: N}`.
  Su física es plana (`speed_navegation`, sic) y la nuestra anidada: el aplanado va de su lado.
- **F6.4 · mip_planner** (lo llama `llm_mission_evaluator`, no el GCS): velocidad por UAV (hoy 5 m/s fija) y
  `vehicleType`; emite `route[]`.
- Colisión 2D/3D según el dominio; el desvío vertical (`detourGenerator`) solo para vehículos aéreos.

**Lo que queda de F4/F5 (sin vehículo que lo use todavía):** ArduPlane / ArduRover (modos en `vehicleStates.js`,
hoy `OTHER`); `vtolState` y `vtol_transition` por `px4_ros2` / `px4_mavros` (su encoder descarta la acción con un
warning); USV/UGV (`pos[2]`, sin takeoff/land; `flightPhase` de superficie).

**F7 · Cooperación:** UAV↔USV (base móvil, aterrizar sobre un vehículo) sobre el DAG de tareas.

## Decisiones tomadas

| Tema | Decisión |
|---|---|
| Vocabulario | **protocol** (transporte + gateway) · **interfaz** (hoy `category`) · **device model** (qué es) · **vehicleType** (cómo se mueve) · **device** (la unidad). "Categoría" NO se reutiliza para el tipo |
| Nombre `category` | Se queda en DB/API/código, documentado como interfaz. Rename a `interface`: aparte (TODO) |
| Capacidades | Feature model: el modelo declara `standard` / `optional`; el device habilita opcionales o deshabilita estándar, nunca habilita lo que el modelo no declara. Efectiva = eso ∩ lo que la interfaz implementa |
| Qué implementa una interfaz | Un catálogo único (`capabilities.yaml`): capacidad → comandos + slots + items de misión. La interfaz implementa una si declara alguno. **Descartado** el `requires:` en las entradas de la interfaz |
| Tipo de vehículo | El modelo declara uno; la interfaz, los que puede servir. Compatibilidad = tipo ∈ `vehicleTypes` ∧ firmware compatible |
| Contenido de misión | Interfaz ∩ vehículo. Lo de equipamiento en `capabilities.yaml` (`mission:`), lo de física en `mission_schema.yaml` (`vehicleTypes`). **Descartado** el `vehicle_types.yaml` |
| Cámaras | Tres capacidades: `camera_stream` (en vivo), `camera` (fotos/video en misión), `file_download`. El gimbal es aparte (cámaras fijas) |
| Validación | Un solo validador en el server para los 3 caminos de carga y para el editor. Lo que el vehículo no puede: **error** (rechaza). Geometría en ala: **warning** (son estimaciones). Device inexistente: warning (la tarea falla y salta sus dependientes, como antes) |
| Altura de misión | `altitude_ref` por tarea, **relativa por defecto** (editor y planner: `calculateLocalOrigin` da `alt: 0`). Un valor desconocido es error, nunca cae en el default |
| VTOL | Despegue `NAV_TAKEOFF` (queda en MC); la transición es una **acción por waypoint**; Auto landing → `NAV_VTOL_LAND`. Fin de misión: lo decide PX4 (`MIS_TKO_LAND_REQ`), sin reescribir el RTL |
| Física | En el modelo (formato de Smart-Flies). El GCS la usa para AVISAR (geometría en ala); la dinámica real la planifica el planner |
| Física en el planner (R5) | El GCS la manda inline (`vehicle`) y tiene prioridad; el planner usa la suya solo si no viene (corre sin el GCS, o el modelo no declara ese bloque). El GCS nunca rellena con defaults |
| Quién vuela un tipo de misión | `vehicleTypes` + `capabilities` en `config/planning/config.yaml`; inspección (línea, torre, vegetación): multirrotor + `autonomous_mission` + `camera`; lidar: multirrotor + `autonomous_mission` + `lidar_3d` |
| Lidar | Dos capacidades **a bordo** (`onboard: true`: el GCS no habla con el sensor, ninguna interfaz la implementa; cuenta si el modelo la declara, nunca en un genérico `'*'`): `lidar_2d` estándar en bcr_bot, `lidar_3d` opcional en x500 (se habilita en la unidad que lo tiene). **Descartado** un slot de telemetría: suscribir una nube de puntos por rosbridge para no mostrarla |
| MAVLink | Interfaz = protocolo + dialecto (`px4_mavlink`, `ardupilot_mavlink`), no dos protocolos |
| Variantes de interfaz | Plano + `extends`; el editor guarda la forma cruda (`/raw`) |

## Historial: lo que no está en el código

- **F1:** `firmwares` es LISTA en la interfaz (mavros traduce MAVLink: `[px4, ardupilot]`); el firmware que CORRE un
  vehículo es del modelo. El relay `_slow` corre en el dron (lo único que ahorra enlace en ROS 1): hoy es el atributo
  `attributes.slow`, que lee `/<device>/slow<topic>`.
- **F2:** el modelo LISTA sus interfaces a mano (la regla deducida le ofrecía `catec` a un x500); la regla queda como
  verificación al arrancar. Los DJI copian física sin verificar; no se pasó a fuvex/catec/px4 (era el M210 copiado).
- **F3:** M600 solo telemetría; catec + fuvex → `external_telemetry`; griffin por `ardupilot_mavlink` solo telemetría.
  Cambio de comportamiento: sin habilitar `rtk` / `gimbal` / `camera_stream`, no se suscriben ni se ofrecen.
- **Altura (primer intento en SITL):** PX4 rechazó la misión del editor ("Takeoff altitude below home altitude!"):
  el encoder mandaba la altura relativa del editor como AMSL (5 m con el home a 488 m). Le pasaba a TODA misión del
  editor por MAVLink; los vuelos anteriores eran scripts con AMSL. En ArduPilot habría volado bajo tierra.
  El R1 de `MAVLINK_PLAN.md` (origen con altitud de los devices) está desactualizado.
- **VTOL en PX4** (leído en `~/PX4-Autopilot` v1.18-dev): `NAV_TAKEOFF` deja el VTOL en MC; antes de pasar a FW
  orienta hacia el SIGUIENTE waypoint (`Mission::handleVtolTransition`); no da el `DO_VTOL_TRANSITION` por cumplido
  hasta estar en FW; los airframes VTOL (SITL incluido) fijan `MIS_TKO_LAND_REQ=2` (sin aterrizaje, se rechaza; un
  RTL no cuenta). La primera misión manual no transicionó por geometría (piernas de ~12 m a 5 m).
- **Valores del `standard_vtol`** (PX4 SITL): `FW_AIRSPD_TRIM` 15 m/s (medido ~17.5 sobre el suelo), `FW_R_LIM` 50°
  → radio 19.2 m, `VT_ARSP_TRANS` 10, `VT_F_TRANS_DUR` 5 s (medido ~3 s), `VT_B_DEC_MSS` 2 m/s², `MPC_XY_CRUISE` 5.
  PX4 no tiene altura mínima en FW: se usan los 20 m de `MIS_TAKEOFF_ALT` de `rc.vtol_defaults`.
- **Validador:** antes, una acción que la interfaz no codificaba se descartaba con un warning; ahora la carga se
  rechaza. Grabar un waypoint desde la posición ya no mete `gimbal` sin la capacidad.
- **F6.1, cambio de comportamiento:** el filtro de tipos de misión ya no es por categoría. Un x500 (`px4_ros2`, antes
  admitido en línea/torre) queda fuera hasta habilitarle `camera`; la inspección de lidar ya no es del M600 (no
  declara `autonomous_mission`: el GCS le rechazaba la carga) sino del x500 con `lidar_3d` habilitado. El skip se
  loguea con el motivo (`info`).

## Riesgos abiertos

| # | Riesgo |
|---|---|
| R1 | El editor de categorías guarda la forma cruda, pero reescribe el archivo y pierde sus comentarios |
| R3 | Los devices existentes están en modelos genéricos o por defecto: el planner no gana nada hasta cargar los reales |
| R4 | MCP y prompts de agentes nombran categorías concretas |
| R5 | El formato de física de Smart-Flies se vuelve un contrato |
| R6 | `extends` en dos niveles (interfaz y modelo) hace difícil saber de dónde sale un valor |
| R8 | Los números del modelo (geometría) son del SITL: un vehículo real con otros parámetros da avisos equivocados hasta verificarlos (pendiente 6) |

## Fuera de alcance

- Rename `category` → `interface` (TODO aparte).
- Datum de altitud y calidad del fix RTK (TODO "Slots de posición satelital").
- Enlace MAVLink por device (TODO aparte).
- Viento, autonomía/batería y trayectorias Dubins exactas en el GCS: son del planner (Smart-Flies).
