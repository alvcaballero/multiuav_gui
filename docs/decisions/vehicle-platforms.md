# Device models, tipos de vehículo y capacidades — plan (CERRADO 2026-10-26)

Objetivo: que la flota pueda ser heterogénea (multirrotor, ala fija, VTOL, USV, UGV) y que cada device diga QUÉ
es (tipo, física, capacidades) por separado de CÓMO se le habla (protocolo, stack, mensajes). Antes las dos
cosas estaban fundidas en la "categoría", y eso producía una categoría por combinación.

**Estado:** F1–F6 hechas. Lo que quedó abierto (pendientes, cabos sueltos de F6, USV/UGV, ArduPlane, cooperación
F7) está en `TODO.md` (raíz de `multiuav_gcs/`). Este archivo queda como registro de lo decidido y por qué; el
código lo cita por fase (`docs/decisions/vehicle-platforms.md › F3`, …). Cómo funciona hoy:
- `server/CLAUDE.md` › ROS Integration: interfaces, **Device models**, **Capabilities**, **Mission content follows
  the vehicle** (incluye la geometría en ala); › Mission Planning System (formato v4, `altitude_ref`); › MAVLink
  Protocol › Missions (VTOL, altura relativa, `test:e2e:vtol`).
- `client/CLAUDE.md` › `components/devices/` y `mission/`, Missions as task graphs, Vehicle state (`vtolState`).

**El modelo mental en una línea:** device = **modelo** (QUÉ es y qué PUEDE: tipo, firmware, capacidades, física)
\+ **interfaz** (`category`: CÓMO se le habla) + **protocolo** (por dónde viaja) + atributos del device
(systemId, slow, overrides de capacidades). **Lo que una misión puede llevar = lo que la interfaz codifica ∩ lo
que el vehículo puede** (capacidades efectivas + `vehicleType`).

## Fases

| Fase | Commits | Qué quedó |
|---|---|---|
| F1.0 / F1 | `0506110`… | Un archivo por interfaz, validador único, `extends`/`abstract`, `firmwares`, `vehicleTypes`, `mission:`, `defaultModel` |
| F2 | …`e615a79` | Device models (`config/devices/device_models/`), `Device.deviceModel`, formulario nombre → tipo → modelo → interfaz → protocolo |
| F3 | …`e615a79`, `eef1e5a` | Capacidades (`capabilities.yaml`, standard/optional/`'*'`, overrides); lista de devices con indicador RTK y filtro por tipo de vehículo |
| fix altura | `ab78624` | `altitude_ref` por tarea: relativa por defecto → `MAV_FRAME_GLOBAL_RELATIVE_ALT` |
| F4/F5 VTOL | `7435b0d`, `b54f9ff` | Eje `vtolState`; acción `vtol_transition`, `NAV_VTOL_LAND`; **volado en SITL** (`npm run test:e2e:vtol`) |
| F5 · Contenido de la misión | `7c2f0e6` | Capacidades dueñas de items de misión (`camera` nueva), `vehicleTypes` en `mission_schema.yaml`, `missionValidator.js` (rechaza en los 3 caminos de carga), `POST /missions/check` + botón Validar, endpoints con `?deviceId=`, formulario del device según capacidad |
| F5 geometría | `04d82ce` | Warnings de geometría en ala (transición, giros, frenado, altura) con los números del modelo; `standard_vtol` con valores de PX4 SITL |
| F6.1 · GCS | `c967c9a`, `4acb9ca` | `planningProfileOf(device)` → `vehicle` en el request a Smart-Flies y en `geodetic-to-xyz` (identidad = nombre + id); tipos de misión por `vehicleTypes` + `capabilities` (`planningRequirements.js`); lidar a bordo (`onboard`) |
| F6.2 · LLM | GCS `d1ae2cb`; mcp_server `87875e2`, `cb4cb02` | `get_devices` con `Vehicle` y `Capabilities`, sin `Category`; selección multirrotor/VTOL + `autonomous_mission` + `camera`; `cruise_speed: vehicle` (cada ruta a `vehicle.speeds.mission`, 5 si no lo declara); el planner LLM recibe `vehicle` recortado (`plannerDeviceView`) |
| F6.2b · eve | `gcs_eevee_assistant` `421865e` | Skill `mission-planning.md` con las mismas reglas; los dos sandbox resuelven `cruise_speed: vehicle` por dron; sin `category` |
| F6.3 · Smart-Flies | `a30793a` (submódulo `llm_planner_gcs/Smart-Flies-MUAVPlanner`); GCS `f5af381` | `resolve_model_data`: `vehicle.physics` / `battery` pisan campo a campo la entrada de `devices.yaml`; lo que falta sale del `dji_M300` con warning; `obstacle_sensor` y `speeds` no se leen |
| F6.4 · mip_planner | — | **Congelado**: lo usa solo `llm_mission_evaluator`; la velocidad heterogénea pide `c[i][j]` → `c[i][j][k]` en el solver y no lo justifica |
| Lidar 3D on-demand | `a836d37`, `690209c` | Slot `lidar_3d_points` de la capacidad `lidar_3d`, suscrito solo mientras alguien lo mira (vista 3D o inspector de mensajes) |

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
| VTOL | Despegue `NAV_TAKEOFF` (queda en MC); la transición es una **acción por waypoint**; Auto landing → `NAV_VTOL_LAND`. Fin de misión: lo decide PX4 (`MIS_TKO_LAND_REQ`), sin reescribir el RTL. El planner LLM NO emite transiciones: un VTOL vuela todo en multicóptero |
| Física | En el modelo (formato de Smart-Flies). El GCS la usa para AVISAR (geometría en ala); la dinámica real la planifica el planner |
| Física en el planner (R5) | El GCS la manda inline (`vehicle`) y tiene prioridad; el planner usa la suya solo si no viene (corre sin el GCS, o el modelo no declara ese bloque). El GCS nunca rellena con defaults |
| Quién vuela un tipo de misión | `vehicleTypes` + `capabilities` en `config/planning/config.yaml`; inspección (línea, torre, vegetación): multirrotor + `autonomous_mission` + `camera`; lidar: multirrotor + `autonomous_mission` + `lidar_3d` |
| Lidar | Dos capacidades **a bordo** (`onboard: true`: el GCS no le da comandos ni items de misión, ninguna interfaz la implementa; cuenta si el modelo la declara, nunca en un genérico `'*'`): `lidar_2d` estándar en bcr_bot, `lidar_3d` opcional en x500. **Descartado** suscribir la nube con el device (~1.8 MB/s por dron para no mostrarla); sí un slot **on-demand** (`lidar_3d_points`) que se abre solo mientras alguien lo mira |
| MAVLink | Interfaz = protocolo + dialecto (`px4_mavlink`, `ardupilot_mavlink`), no dos protocolos (comparten `MavlinkServer`, enlaces, puerto UDP y System IDs) |
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
  El R1 de `mavlink.md` (origen con altitud de los devices) está desactualizado.
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
- **F6.2b:** la vista recortada del device en eve se copia por JSON: toon encodea `null` un objeto de otro realm.
- **F6.3:** un primer intento leía `vehicle.category`, que NO existe (`planningProfileOf` no lo lleva) y rompía todo
  request con `vehicle`; se revirtió. `category` desconocida con `vehicle` completo → sin fallback ni warning. Un
  campo de física que no es número positivo se ignora con warning; los warnings salen en la respuesta de
  `/mission_request` y en el YAML del plan.

## Riesgos que quedan

| # | Riesgo |
|---|---|
| R5 | El formato de física de Smart-Flies se vuelve un contrato: cambiar una clave allá rompe el `vehicle` del GCS |
| R6 | `extends` en dos niveles (interfaz y modelo) hace difícil saber de dónde sale un valor |

Los demás riesgos pasaron al TODO como tareas: editor que pierde comentarios (R1), modelos reales sin cargar (R3),
categorías nombradas en MCP/prompts (R4, con el rename), números de SITL contra vehículos reales (R8, leer
parámetros por MAVLink).
