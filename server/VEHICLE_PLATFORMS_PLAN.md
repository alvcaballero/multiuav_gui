# Device models, tipos de vehículo y capacidades — plan

Objetivo: que la flota pueda ser heterogénea (multirrotor, ala fija, VTOL, USV, UGV) y que cada
device diga QUÉ es (tipo, física, capacidades) por separado de CÓMO se le habla (protocolo, stack,
mensajes). Hoy las dos cosas están fundidas en la "categoría" de `devices_msg.yaml`, y eso produce
una categoría por combinación (las cuatro M210: ROS noetic/melodic × con/sin RTK).

## Diagnóstico (medido, 2026-10-24)

- La categoría es una INTERFAZ, no un vehículo: `dji_M210_noetic` vs `dji_M210_melodic` difieren en
  UNA línea (`messageType` de `battery`); `px4_mavlink` vs `ardupilot_mavlink` solo en el tag de
  dialecto y un subscriber de gimbal. `px4_mavros` lleva tanto un cuadricóptero como `standard_vtol`
  (`infra/Simulation-PX4.md`).
- El tipo de vehículo no existe como dato; se asume multirrotor implícitamente en: el ícono
  (`mapIconKey`, switch por nombre de categoría), `detourGenerator` (esquiva "por arriba"),
  `mip_planner` (`ALTITUDE_STAGGER`, distancia euclídea), `mavlinkFlightMode` (solo ArduCopter),
  `mode_landing: Hover`.
- `config/devices/devices.yaml` ya tiene física por "categoría" (masa, rotores, batería, velocidades)
  pero el server solo lee su bloque `files`: el resto es dato muerto, duplicado por interfaz. Quien sí
  consume esos parámetros es `Smart-Flies-MUAVPlanner` (`modules/uav.py`: modelo de energía, `Min.
  Radius`, Dubins, `NAV_VTOL_TAKEOFF` con heading de transición).
- RTK (resuelto aparte, commits `9ccb2e1..1c7a3c0`): no era una categoría sino un segundo slot de
  posición; se arbitra en runtime por `priority`/`fresh_ms`. Ver TODO "Slots de posición satelital".

## Decisiones tomadas

| Tema | Decisión |
|---|---|
| Vocabulario | **protocol** (transporte + gateway) · **interfaz** (hoy `category`: protocolo + stack/dialecto + slot→topic/messageType) · **device model** (qué es) · **vehicleType** (cómo se mueve) · **device** (la unidad) |
| Nombre `category` | Se queda en DB/API/código, documentado como interfaz. Rename `category`→`interface` en otra fase (TODO) |
| "Categoría" para el tipo | NO se reutiliza la palabra: cada referencia existente pasaría a mentir sin error |
| Modelo de capacidades | **Híbrido (feature model)**: el modelo declara cada capacidad `standard` u `optional`; el device habilita opcionales y puede **deshabilitar explícitamente** una `standard` (p.ej. gimbal roto). Nunca habilita algo que el modelo no declara |
| Capacidad efectiva | `(standard ∪ opcionales habilitadas) − deshabilitadas`, ∩ lo que la interfaz implementa cuando la capacidad depende de un endpoint. El runtime CONFIRMA (watchdog, HEARTBEAT), no configura |
| Tipo de vehículo | El **modelo** declara uno (`vehicleType`); la **interfaz** declara la lista que puede servir (`vehicleTypes`) |
| Compatibilidad modelo↔interfaz | `model.vehicleType ∈ interface.vehicleTypes` **y** `interface.firmware ∈ model.firmware`. Sin listas a mano: una interfaz PX4 nueva sirve a todo modelo PX4 |
| Variantes de interfaz | **Plano + `extends`**: `dji_M210_melodic: { extends: dji_M210_noetic, ... }`. El device guarda UNA key (como hoy); el "selector de comunicación" es una agrupación en la UI por la raíz del `extends` |
| MAVLink | Interfaz = **protocolo + dialecto** (`px4_mavlink`, `ardupilot_mavlink`). NO dos protocolos: comparten `MavlinkServer`, enlaces, puerto UDP y espacio de System IDs. Lo propio de cada airframe (x500, VTOL) es un device model |
| Slots según hardware | Entradas de la interfaz con `requires: { capability }` o `requires: { vehicleType }`: el slot existe (se suscribe, el watchdog lo espera) solo si el device lo tiene. Es estático; lo que cambia en vuelo (RTK) se arbitra en runtime |
| Modelos genéricos | Sí: `generic_multirotor`, `generic_fixed_wing`, `generic_vtol`, `generic_usv`, `generic_ugv` (tipo sin física ni opcionales). Un device sin modelo propio usa el genérico de su tipo |
| Computadora a bordo | Dato informativo de la interfaz (`control: autopilot \| companion`). No gatea nada: lo que habilita ya se deduce de los endpoints |
| Física por modelo | Sale de `devices.yaml` (formato compatible con lo que lee Smart-Flies). El GCS publica restricciones (`min_turn_radius`, velocidades, batería); la dinámica la valida el planner, no el GCS |

## Modelo de datos

```
Device ──► category (interfaz)   devices_msg.yaml     protocolo + dialecto/stack, slots, vehicleTypes, firmware, extends
       └─► deviceModel           device_models.yaml   vehicleType, firmware[], física, capabilities{standard|optional}, extends
       └─► attributes.capabilities                     { rtk: true, gimbal: false }  (ATTRIBUTE_GROUPS)
vehicle_types.yaml    multirotor | fixed_wing | vtol | usv | ugv: altitud, puede quedarse quieto, tipos de wp válidos
capabilities.yaml     catálogo: label + qué comandos/slots la implementan (o ninguno: solo hardware, p.ej. camera_thermal)
```

```yaml
# devices_msg.yaml (interfaz)
dji_M210_noetic:
  firmware: dji
  vehicleTypes: [multirotor]
  control: companion
  subscribers: { ... }
dji_M210_melodic:
  extends: dji_M210_noetic
  subscribers:
    battery: { messageType: 'aerialcore_common/BatteryState_melodic' }
px4_mavlink:
  firmware: px4
  vehicleTypes: [multirotor, vtol, fixed_wing]
  control: autopilot
  subscribers:
    gimbal:     { messageType: 'mavlink/GIMBAL_DEVICE_ATTITUDE_STATUS', requires: { capability: gimbal } }
    vtol_state: { messageType: 'mavlink/EXTENDED_SYS_STATE', requires: { vehicleType: vtol } }

# device_models.yaml
dji_M210:
  vehicleType: multirotor
  firmware: [dji]
  physics: { mass: 7.2, rotors: 4, ... }      # de devices.yaml
  battery: { number: 2, capacity: 80000, cells: 4, voltage_per_cell: 3.9 }
  capabilities: { camera_rgb: standard, gimbal: standard, file_download: standard, rtk: optional }
x500:
  vehicleType: multirotor
  firmware: [px4, ardupilot]
  capabilities: { gimbal: optional, camera_rgb: optional }
standard_vtol:
  vehicleType: vtol
  firmware: [px4]
  limits: { min_turn_radius: 50 }
```

## Fases

Cada fase tiene un dominio propio y termina con tests en verde (`npm test`).

### F1.0 · Un archivo por interfaz + validación de cada archivo

Refactor sin cambio de comportamiento, antes de `extends`. Motivo: `devices_msg.yaml` tiene 658 líneas
y 16 categorías, y el editor de categorías (`writeDataFile`, `models/category.js:255`) reescribe el
archivo ENTERO al guardar, así que editar una categoría borra los comentarios de las 16.

- `config/devices/interfaces/<key>.yaml`, carpeta plana. El **nombre del archivo es la key** (el sistema
  de archivos garantiza que sea única) y el contenido es el bloque de la categoría, sin el nivel de la key.
  Los prefijos (`dji_*`, `px4_*`, `ardupilot_*`) los agrupan en orden alfabético.
- Solo `categoryModel` cambia: lee la carpeta en vez del archivo (`models/category.js:12`) y escribe UN
  archivo por categoría (`:255`). Nadie más usa la ruta.
- Crear, borrar o renombrar desde la API pasa a ser una operación de archivo: la key se valida con una
  regex (`^[A-Za-z0-9_]+$`), nunca se arma una ruta con lo que manda el cliente sin validarlo (`../`).
- **Autodescubrimiento**: cada `*.yaml` de la carpeta es una interfaz; no hay índice ni registro
  aparte. Otras extensiones (`.bak`, `.orig`) se ignoran. Un nombre de archivo que no cumple la regex
  de key es un error reportado, no se ignora en silencio.
- Test golden: la categoría resuelta de cada una de las 16 es idéntica antes y después del split.
- Misma convención para los device models en F2 (`config/devices/device_models/<key>.yaml`).

**Validación de cada archivo** (un validador único, que usan el boot, la API y los tests):

- Hoy ya se valida al bootear: que la key esté en el catálogo (`validateDevicesMsgKeys`), que el par
  `(slot, messageType)` tenga decoder, los campos legacy (`findLegacyTypeFields`) y los comandos
  (`validateCommands`). **Falta la forma de cada entrada**: un `nmae:`, un `messageTyp:` o
  `rate_hz: 'dos'` pasan sin aviso. Caso real: `griffin.subscribers.speed` tiene `name: ' '`.
- Schema zod estricto (`schemas/zod/interface.js`):
  - bloques permitidos (`subscribers`, `publishers`, `services`, `actions`, `heartbeat`, `commands`, más los
    nuevos de F1: `extends`, `firmware`, `vehicleTypes`, `control`);
  - cada entrada con `messageType` obligatorio;
  - `name` obligatorio y empezando con `/` en formato `ros_msg`, y ausente en `mavlink`;
  - params conocidos con su tipo (`watchdog`, `timeout_s`, `rate_hz`, `scale`, `unit`, `heading_offset`,
    `priority`, `fresh_ms`; `requires` en F3);
  - cualquier key desconocida es un error.
  Más adelante, cada decoder podría declarar en `register()` los params que acepta, en vez de mantener la lista a mano.
- Un error de parseo YAML se reporta con el nombre del archivo y la línea.
- Las validaciones que ya existen se mueven a este mismo validador, para que haya un solo punto de entrada.
- **Política ante un archivo inválido**: esa interfaz NO se carga (`logger.error` con todos los problemas
  juntos) y las demás siguen andando. Los devices que la usan caen en el error de boot "categoría
  inexistente" (`6a1fa40`). Con `LOG_LEVEL=debug` se lanza la excepción, como hoy.
- `POST`/`PUT /api/category/:category` pasan por el MISMO validador ANTES de escribir: `400 { errors }` y
  nunca queda en disco un archivo inválido. Reemplaza a `rejectLegacyTypeFields` (`controllers/category.js`).
- `test/test-devices-msg.js` corre el validador sobre todos los archivos de la carpeta: un archivo mal
  escrito rompe `npm test`, no el boot en campo.

#### Estado F1.0 (2026-10-24): hecho (`0506110`)

16 archivos, comentarios conservados, golden idéntico. La primera corrida del validador solo encontró
`griffin.subscribers.speed` (`name: ' '`), que se borró: el topic real no se conoce. El editor sigue
sin poder CREAR una categoría (`EditItemView` hace `POST /api/category` sin nombre; deuda anterior).

### F1 · Interfaces: `extends`, `vehicleTypes`, `firmware`

- `categoryModel` (único lector de las interfaces) resuelve `extends` en DOS pasadas: carga todas en
  crudo y después resuelve, así que el orden de los archivos no importa. Merge profundo por bloque/slot.
  Al bootear detecta: padre inexistente, ciclo, y padre inválido (sus hijos no se cargan y el error
  lo dice explícitamente).
- `abstract: true`: una base que solo sirve para heredar (p.ej. lo común de OSDK). No aparece en el
  selector del device, y `POST`/`PUT /api/devices` la rechazan.
- Las referencias por NOMBRE que quedan se validan al bootear contra las interfaces cargadas:
  `config/planning/config.yaml` (hasta F6, donde pasa a filtrar por `vehicleType`/capacidades).
  `CATEGORY_PROFILE_OVERRIDES` (`missionSymbols.js`) pasa a ser el campo `mission:` de la interfaz,
  que ya está soportado (`category.js:143`).
- Campos nuevos `firmware`, `vehicleTypes` y `control` en cada categoría, validados contra
  `VEHICLE_TYPE` (`config/status.js`) al bootear. Hay que completarlos en todas las categorías existentes.
- `dji_M210_melodic` pasa a `extends: dji_M210_noetic`; revisar el mismo patrón en el resto (p.ej.
  `px4_mavros_slow`).
- **Riesgo R1**: el editor de categorías (Settings › Category) reescribe el YAML (`writeDataFile`).
  Tiene que editar y guardar la forma CRUDA (con `extends`), no la resuelta, o aplana las variantes.
- Cliente: el selector de categoría agrupa por la raíz del `extends` ("M210 · ROS Noetic / Melodic").

#### Estado F1 (2026-10-24): hecho

- `extends` (con `slot: null` para quitar una entrada heredada), `abstract`, `firmwares`, `vehicleTypes`,
  `control` y `mission:` (reemplaza a `CATEGORY_PROFILE_OVERRIDES`). `dji_M210_melodic` = `extends:
  dji_M210_noetic` + su batería (5 líneas en vez de 57). `px4_mavros_slow` = `extends: px4_mavros` + los
  topics `/slow/*` del relay + sus dos tipos `multiuav_interfaces/*`.
- **`firmwares` es una LISTA** en la interfaz: con qué firmwares funciona correctamente (mavros traduce
  MAVLink, así que las interfaces mavros listan `[px4, ardupilot]`). El firmware que CORRE un vehículo es
  del device model / device (F2). Por eso no hace falta `unknown`: lo que no se sabe es del vehículo.
- Vocabularios nuevos en `config/status.js`: `VEHICLE_TYPE` (+ `flapping_wing` para griffin), `FIRMWARE`
  (+ `nav2` para el UGV, `custom` para sjtu_drone: plugin de Gazebo con sus PIDs).
- Identidad declarada por el usuario: px4_mavros(_slow) = multirotor (x500 por mavros); simulationFB = M300
  por robofleet (dji); agv = UGV con Nav2; sjtu y catec multirotor; fuvex ala fija; griffin ornitóptero.
- Se borraron las entradas `state_machine` de las interfaces (M210, M300, px4_mavros, catec): eran una
  prueba. Los decoders siguen registrados.
- `config/planning/config.yaml` nombraba `'px4'` (no existe) → `'px4_ros2'`. El chequeo al bootear lo encontró.
- **El relay `_slow` corre en el dron**: es lo único que ahorra el enlace en ROS 1 (rosbridge corre junto al
  GCS; `throttle_rate` de rosbridge solo descarta del lado del GCS, después de que los mensajes cruzaron).
  Falta: cambiar el launch del relay en el dron de `/gcs/*` a `/slow/*`, y declarar `rate_hz` en las
  entradas del `_slow` (para el timeout del watchdog) cuando se sepa qué tasa entrega.
- Pendiente para F2: el device `vtol_1` (DB local) usa `px4_mavros_slow`, que declara solo `multirotor`. Si
  su modelo es VTOL, la compatibilidad lo va a rechazar: agregar `vtol` a la interfaz o moverlo.
- Cliente: el editor carga `/raw` y avisa de quién hereda; el selector de categoría del device lista cada
  variante debajo de su raíz ("dji_M210_melodic (variant of dji_M210_noetic)").

### F2 · Device models

- `config/devices/device_models.yaml` (la física sale de `devices.yaml`; los presets de `files` se quedan
  donde están o se mueven a su propio archivo) y `vehicle_types.yaml`. Modelos genéricos por tipo.
  `extends` también en modelos (variantes de payload).
- Columna `Device.deviceModel`. Migración aditiva e idempotente en `common/migrations/`: a los devices
  existentes se les asigna el genérico del primer `vehicleType` de su categoría.
- `categoryModel.deviceProblem` valida la compatibilidad modelo↔interfaz en `POST`/`PUT /api/devices`.
  Con ese mismo patrón hoy el protocolo filtra la categoría.
- `GET /api/device-models` (y `?category=` → los compatibles).
- Cliente: el formulario del device pide modelo → protocolo → interfaz (filtrada por las dos cosas).
  El ícono del mapa y el modelo 3D salen del modelo/`vehicleType`; se van `mapIconKey` y el switch de `ModelLoader.jsx`.
- MAVLink: el HEARTBEAT (`MAV_TYPE`, `autopilot`) se compara con el modelo declarado. Si no coinciden,
  warning (no se rechaza).

#### Estado F2 (2026-10-24): hecho

- 17 archivos en `config/devices/device_models/`: un `generic_<tipo>` por cada uno de los 6 tipos,
  `_dji_rotorcraft` (abstracto) → `dji_M210`, `dji_M300`, `dji_M600`; `x500` (px4, ardupilot),
  `standard_vtol` (px4), `fuvex` (ala fija, px4), `catec` (px4), `griffin` (ornitóptero, ardupilot),
  `sjtu_drone` (custom), `bcr_bot` (ugv, nav2).
- El firmware que CORRE un vehículo está en su modelo; la interfaz dice cuáles puede manejar (mavros: los dos).
- `defaultModel` en las interfaces (px4_* y mavlink → x500; las DJI → su modelo; agv → bcr_bot…).
  `px4_mavros` (y por herencia `_slow`) sirve ahora `[multirotor, vtol]`.
- Los datos físicos de los DJI son copias sin verificar del viejo `devices.yaml` (TODO). Se borraron de
  `devices.yaml` (quedan `camera` y los presets de `files`). No se pasaron a fuvex/catec/px4: eran el mismo
  M210 copiado (4 rotores en un ala fija).
- **Revisado tras usarlo (2026-10-24):** el formulario va en el orden de las dependencias,
  nombre → **modelo** → interfaz → protocolo → System ID. El modelo LISTA sus `interfaces` a mano
  (opción A: la regla deducida ofrecía `catec` a un x500, porque es mavros multirrotor); la regla
  queda como verificación al arrancar de esa lista. El protocolo se deduce del formato de mensaje de
  la interfaz (MAVLink → solo mavlink, auto). Sin `protocols:` en la interfaz: lo que sobra
  (`simulationFB` solo robofleet) se arregla alineando robofleet con ROS (TODO).
- Mapa, lista de devices y 3D se dibujan por `device.platform` (adiós a los `switch` por categoría).
  Arreglado de paso: `px4` se dibujaba como avión.
- Migración: columna `Device.deviceModel` (aditiva) + relleno en el boot con el default de su
  interfaz. Probado en una copia de la DB local: 16 devices, idempotente. **`vtol_1` queda como `x500`**
  (default de `px4_mavros_slow`): hay que pasarlo a `standard_vtol` desde el formulario.
- No entró (a propósito): `vehicle_types.yaml` con la semántica por tipo (F4/F5) y mandarle el modelo a
  Smart-Flies, que hoy indexa su propio `files/devices.yaml` por NOMBRE DE CATEGORÍA (F6).

### F3 · Capacidades

- `capabilities.yaml` (catálogo) y el bloque `capabilities` del modelo, validados al bootear. Una capacidad
  declarada que la interfaz no implementa da warning, no error (combinación válida: no aparece).
- `attributes.capabilities` del device como grupo de `ATTRIBUTE_GROUPS` (`appliesTo`: el modelo tiene
  opcionales o standard deshabilitables). La validación estricta y la poda al editar ya existen.
- `requires: { capability | vehicleType }` en las entradas de la interfaz → suscripción y watchdog
  solo de los slots que el device tiene (mismo punto que `subscribesCamera`).
- **Riesgo R2**: cambiar capacidades cambia las suscripciones. `models/devices.js` (~l.308) solo vuelve a
  suscribir cuando cambia el nombre o la categoría, y lo condiciona a `protocol === ROS` con el protocolo
  del REQUEST: un `PUT` parcial no vuelve a suscribir. Verificar y arreglar acá.
- `shouldDownloadFiles` lee la capacidad `file_download`. `/api/commands/types` y `sendCommand` excluyen
  los comandos de una capacidad deshabilitada.
- `DevicesModel.getAll` expone las capacidades efectivas y el cliente las muestra como chips (pocas),
  con un filtro por capacidad.

#### Estado F3 (2026-10-25): hecho, salvo chips y filtro en la UI

- **Cambio de diseño respecto de lo de arriba:** no hay `requires:` en las interfaces. Un catálogo único
  (`capabilities.yaml`) dice qué comandos y slots implementan cada capacidad; la interfaz implementa una
  si declara alguno. Las interfaces no cambiaron (salvo las uniones de abajo).
- Capacidades por modelo (decididas por el usuario): M210/M300 misiones + gimbal + cámara + descarga +
  obstáculos, RTK opcional; **M600 solo telemetría** (y su interfaz sin servicios); x500/standard_vtol
  misiones + vuelo + descarga, gimbal y cámara opcionales; catec `threat_detection`; fuvex nada; griffin
  `humidity_sensor`; bcr_bot navegación + cámara; sjtu nada; genéricos `'*'`.
- **catec + fuvex → `external_telemetry`** (batería `sensor_msgs/BatteryState`, la de fuvex); el `threat` es
  capacidad de catec. Migración: `UPDATE Devices SET category='external_telemetry'` (idempotente).
- **griffin por `ardupilot_mavlink`: solo telemetría** (la interfaz sirve ahora `flapping_wing`; griffin no
  tiene `flight_control` ni `autonomous_mission`, así que no se le ofrece ni manda ningún comando de vuelo).
- R2 arreglado: `editDevice` re-suscribe por el protocolo GUARDADO y ante cualquier cambio del que
  dependen las suscripciones (nombre, interfaz, modelo, attributes, cámara), con la cámara nueva.
- Cliente: casillas de capacidades bajo el modelo (solo se guardan las diferencias con el modelo);
  los botones de amenaza de `StatusCard` miran `threat_detection`, no el nombre `catec`.
- **Cambios de comportamiento a tener en cuenta:** un M210/M300 ya no se suscribe a `position_rtk` hasta
  habilitar `rtk` en el device; un x500 ya no ve Gimbal ni se suscribe a la cámara hasta habilitar
  `gimbal` / `camera_stream`.
- Pendiente: chips con las capacidades en la lista de devices y filtro por capacidad.

### F4 · Telemetría según tipo de vehículo

- Eje nuevo `vtolState` (MC / FW / transición) en `position.attributes`, en el vocabulario de
  `config/status.js` + espejo en el cliente (`test-vehicle-state-mirror.js`).
- Adaptadores de modo en `vehicleStates.js`: PX4 FW/VTOL, ArduPlane, ArduRover. Hoy caen en `OTHER`.
- `flightPhase` para superficie/tierra (no hay "airborne").

### F5 · Misión según tipo de vehículo

- `vehicle_types.yaml` dice qué tipos de wp acepta cada tipo: `vtol_takeoff` (con heading de transición,
  como en Smart-Flies), `vtol_transition`, `vtol_land`. `validateTaskGraph` rechaza un wp que el tipo no admite.
- USV/UGV: `pos[2]` se ignora o se rechaza (decidir en la fase). No hay takeoff/land.
- El perfil de `mission_schema.yaml` pasa a ser la intersección encoder ∩ `vehicleType`: sin `Hover` en
  `mode_landing` para ala fija, sin `stay` donde no puede quedarse quieto.
- Encoders MAVLink para VTOL (`NAV_VTOL_TAKEOFF`/`DO_VTOL_TRANSITION`/`NAV_VTOL_LAND`).

### F6 · Planificación y seguridad

- Los planners reciben el modelo: física, batería, `min_turn_radius`, velocidades. Primero Smart-Flies
  (ya consume ese formato), después `mip_planner`. **Riesgo R5**: congelar el contrato de parámetros con Smart-Flies.
- Colisión 2D/3D según el dominio. El desvío vertical (`detourGenerator`) solo para vehículos aéreos.
- `vehicleType` y capacidades efectivas en el schema MCP y en los prompts de los agentes (`planner.md` es
  la fuente de verdad, `plannerFast.md` en sync). **Riesgo R4**: los prompts nombran categorías.

### F7 · Cooperación

- UAV↔USV (base móvil, aterrizaje sobre un vehículo) sobre el DAG de tareas existente.

## Riesgos abiertos

| # | Riesgo |
|---|---|
| R1 | El editor de categorías aplana el `extends` al guardar (y ya pierde los comentarios del YAML) |
| R2 | Un `PUT` parcial de device no vuelve a suscribir (sin verificar; ver F3) |
| R3 | Los devices existentes quedan en modelos genéricos: el planner no gana nada hasta cargar los modelos reales |
| R4 | MCP y prompts de agentes nombran categorías concretas |
| R5 | El formato de física de Smart-Flies se vuelve un contrato |
| R7 | El schema estricto puede rechazar params que hoy se usan sin estar documentados: la primera corrida del validador sobre las 16 interfaces va a mostrar deuda (como `griffin.speed`). Arreglarla en F1.0, no relajar el schema |
| R6 | `extends` en dos niveles (interfaz y modelo) puede hacer difícil saber de dónde sale un valor: `GET` debe poder devolver la forma resuelta y la cruda |

## Fuera de alcance

- Rename `category` → `interface` (TODO aparte).
- Datum de altitud y calidad del fix RTK (TODO "Slots de posición satelital").
- Enlace MAVLink por device (TODO aparte).
- Validar dinámica de vuelo en el GCS (radios de giro, stall): es trabajo del planner.
