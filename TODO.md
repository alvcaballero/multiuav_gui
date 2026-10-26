# TODO

El ÚNICO backlog de `multiuav_gcs/` (server, client y lo que el GCS necesita del dron). Reglas en `CLAUDE.md` ›
Docs y backlog: lo terminado se borra (queda en `git log`), un plan cerrado no guarda pendientes, nada de TODO
gitignored. Decisiones y porqués: `docs/decisions/`. Planes activos: `docs/plans/`.

Triage contra el código: 2026-10-26. Las rutas son relativas a `multiuav_gcs/`.

---

## CRÍTICO — Seguridad de vuelo

- [ ] **No armar sobre una misión que PX4 marcó inválida** (Server)
  - Hoy el GCS arma, PX4 rechaza el modo (`TEMPORARILY_REJECTED`) y desarma solo
  - El dato existe: `mission_result.valid` por MAVLink. Esperarlo después de la carga y no comandar si es inválida

- [ ] **No dejar borrar un device que está volando** (Client)
  - `client/src/components/devices/StatusCard.jsx:117-135`: el botón de borrar está pegado a los demás, sin texto
    ni tooltip; `RemoveDialog` (`:394`) es una confirmación genérica que no mira `armState` / `flightPhase`
  - Moverlo a un menú y bloquearlo (o doble confirmación) si el dron está armado o en vuelo; etiquetar las acciones

- [ ] **Probar KillMotors en SITL** (Server)
  - `COMPONENT_ARM_DISARM` 21196 (`server/models/commands/mavlinkCommand.js:68`) nunca se probó en vuelo, en
    ninguno de los dos firmwares. Es el comando de emergencia

---

## ALTO — Cabos sueltos de la planificación por vehículo

- [ ] **Commitear los punteros de los submódulos en `llm_planner_gcs`**
  - `Smart-Flies-MUAVPlanner` (`a30793a`), `mcp_server` (`cb4cb02`) y `multiuav_gcs` están modificados en el repo
    raíz sin commitear. El checkout bueno de Smart-Flies es `llm_planner_gcs/Smart-Flies-MUAVPlanner`

- [ ] **Mergear `feat/vehicle-aware-planning` del `mcp_server`** (`87875e2`, `cb4cb02`)
  - En ese repo el usuario commitea directo en `main`; la rama la abrió una sesión

- [ ] **Probar un request real GCS → Smart-Flies con `vehicle`**
  - Los tests de Smart-Flies pasan (`uv run pytest tests/`, 20) pero nunca se corrió el camino completo

- [ ] **eve: el asistente delega en el planner sandbox viejo** (eve repo)
  - Delega en `subagents/planner_sandbox` (emite `route[]`, sin dependencias); el que emite `tasks[]`
    (`agents/planner_sandbox`) es un agente de prueba
  - Su `DEFAULT_PARAMS` lleva `mode_gimbal: 0`, que el validador del GCS rechaza en un dron sin `gimbal`

- [ ] **Los planners emiten `route[]`, no `tasks[]`** (mip_planner + mcp_server)
  - `mip_planner/core/planner.py:603` y `MissionSchemaXYZ` (`mcp_server/src/schemas/missions.ts:76`): el flujo
    automático no tiene dependencias entre tareas hasta que emitan `tasks[]`

---

## Server — Misiones

- [ ] **Mostrar los warnings del validador al cargar**, no solo con el botón Validar (Server + Client)

- [ ] **Validar mientras se edita** (con debounce) y marcar el waypoint con error dentro del editor (Client)
  - Hoy: lista de problemas + click

- [ ] **Radio de aceptación de waypoint configurable**
  - `server/models/mission/missionWpTracking.js:16`: `WP_REACHED_THRESHOLD_M = 2` fijo. Un ala fija no pasa a 2 m
    de un WP. A `config/config.js` o, mejor, al device model

- [ ] **Colisiones respetando el grafo de tareas**
  - `server/models/collision/collisionDetector.js` ignora `depends_on`: dos tareas secuenciales dan un falso
    conflicto entre UAVs
  - Colisión 2D/3D según el dominio; el desvío vertical (`detourGenerator`) solo para vehículos aéreos

- [ ] **`MissionTask` con hora de inicio real**
  - `initTime` es la creación (`server/schemas/database/missionTask.model.js:57`): los reportes (eventos,
    trayectoria) de dos tareas de un mismo device se solapan

- [ ] **Leer los parámetros del vehículo por MAVLink** (`PARAM_REQUEST_READ`)
  - Para verificar el device model contra el vehículo real. Los números de geometría del `standard_vtol` son de
    PX4 SITL: un vehículo real con otros parámetros da avisos equivocados. Con eso la geometría en ala podría pasar
    de warning a error

- [ ] **Rechazar en el server `camera[]` / `files[]` de un device sin `camera_stream` / `file_download`**
  - Hoy solo se ocultan en el formulario, sin borrarlos

- [ ] **Abort y Pause como commands compuestos del server** (Server + Client)
  - `client/src/pages/device/useMissionActions.js` encadena StopMission + ReturnHome y guarda `paused` en local:
    la UI y el LLM no ven lo mismo. Hace falta un command compuesto y un estado `paused` en la tarea

---

## Server — Comandos

- [ ] **Sin telemetría de gimbal no se distingue de un gimbal centrado**
  - El default `gimbal: [0,0,0]` (`server/models/positions/positionReducers.js:69`) hace que `gimbalRequest`
    (`server/models/commands/commandRegistry.js:71`) mande 0 en el eje que falta. Usar `null` hasta la primera
    muestra y, sin ella, exigir los dos ejes

- [ ] **Orden de ejes de la telemetría del gimbal DJI**
  - `server/models/msgDecode.js:222-229` copia `vector.x/y/z` de `Vector3Stamped` como pitch/roll/yaw sin mapear.
    Verificar contra dji_osdk_ros y psdk_ros2; si está mal, el mapeo va como param del subscriber (como `unit`)

- [ ] **CameraFileDownload: params que el encoder lee y el schema rechaza**
  - `server/models/msgEncode.js:113-127` lee `payload_index`, `file_type`, `delete_after_download`, que no están en
    `config/devices/command_schema.yaml`: si alguien los manda, `validateAttributes` los rechaza

- [ ] **Validar las keys de una interfaz**
  - La misma key en dos bloques (services/actions/publishers) no se detecta (`server/models/category.js:72`,
    `models/interfaces/interfaceCatalog.js`): así entró el VehicleCommand duplicado
  - `commands.<Type>` acepta cualquier key (`schemas/zod/interface.js`, `z.record(z.unknown())`;
    `validateCommandOverrides` solo mira `params`): un `vai:` o `parms:` se ignora sin aviso. Permitir solo
    `params` y `via`

- [ ] **Límites de los commands**
  - Los `# TODO límites` de Takeoff `altitude` y MoveRelative `dx/dy/dz/dyaw` en `command_schema.yaml`
    (decisión del usuario)
  - Gimbal: ninguna interfaz declara `commands: { Gimbal: … }` y el cliente hardcodea -90/0
    (`client/src/pages/device/manualControl.js:15-16`): leerlos de `/api/commands/types`. Dos drones de la misma
    interfaz con gimbals distintos necesitan override por modelo o device

- [ ] **Verificar los params `SUPUESTO` contra los .srv**
  - `SetupCamera` (dji_osdk_ros/SetupCameraStream), `CameraFileList` (dji_osdk_ros/FileList), `UploadMission`
    (psdk UploadWaypointV2Mission) en `command_schema.yaml`

- [ ] **El mensaje de error se pierde con ROS deshabilitado**
  - `server/controllers/ros.js:236,243,249` devuelven `{state, message}` y `server/models/commands/commands.js:118`
    lee `response.msg`: el Event queda con `undefined`

- [ ] **MCP `get_available_commands` usa un endpoint muerto** (Server + mcp_server)
  - `mcp_server/src/tools/command.ts:85` pide `GET /commands/send`, que siempre devuelve `[]`. Pasar a
    `/commands/types?deviceId=` y borrar `routes/commands.js:8`, `controllers/commands.js:9` y `getSaveCommands`

- [ ] **Preconditions por estado** (diferido a propósito)
  - Arm solo desarmado, Takeoff solo en tierra. Hoy no hay nada en `models/commands/`

- [ ] **Migrar `attributes.action` viejos** (condicional)
  - Solo si un reporte empieza a agrupar o filtrar por `action`; hoy solo se muestra

---

## Server — Chat (orquestador legacy)

eve tiene su propio plan activo: `docs/plans/eve.md`.

- [ ] **MCP caído deja el chat sin tools y sin aviso** (P1)
  - `MessageOrchestrator.initializeLLMProvider` (`server/models/chat/chat.js:86-95`) solo loguea si
    `mcpClient.connect()` falla; `getToolsForProvider` (`:818`) devuelve `[]`, así que `executeTool` (el único que
    reconecta) nunca se llama. Un LLM que falla deja `llmHandler=null` (`:80-83`) y nadie reintenta
  - Retry con backoff, estado `ready|degraded|failed` en `isReady()`, avisar cuando un turno corre sin tools,
    503 en `controllers/chat.js:46-49`

- [ ] **`mcpClient.executeTool`: reintento recursivo sin cota** (P1)
  - `server/models/chat/mcpClient.js:277` se llama a sí mismo tras reconectar y `connect()` resetea
    `reconnectAttempts` (`:83`): el límite de 5 nunca corta. `includes('session')` (`:264`) toma por error de
    conexión cualquier mensaje que diga "session"
  - Parámetro `attempt` (un reintento tras reconectar) y match estricto de errores de conexión

- [ ] **Sin deadline por turno: un LLM colgado bloquea el chat** (P1)
  - El lock del chat dura toda la recursión (`chat.js:307`, `:319`). Solo `openaiCompatibleHandler.js:95` tiene
    timeout (8 min); OpenAI, Anthropic y Gemini usan el default del SDK. Los subagentes no tienen deadline
    (`subAgentManager.js:95`, `:145`)
  - `timeout` en cada cliente SDK, deadline por turno (AbortSignal) que libere el lock, TTL de subagentes

- [ ] **`addMessage` traga fallos de persistencia** (P2)
  - `server/models/chat/chatHistoryManager.js:144-148` loguea y devuelve el ítem, que se emite como guardado; tras
    un reinicio desaparece y puede dejar `function_call` huérfanos. Para `function_call*`, fallar el turno

- [ ] **`allowedTools`: tres estados y sin validación** (P2)
  - `chat.js:830-844` distingue `null`/`undefined`/`[]`; `turnContext.js:60` (`??`) hace que un `null` explícito
    caiga al perfil del agente; un nombre mal escrito filtra en silencio. Tipo explícito
    `{mode: 'all'|'none'|'restricted', tools}` y aviso si un nombre no existe

- [ ] **Borrar un chat deja `ChatToolApproval` huérfanos** (P2)
  - `chatHistoryManager.js:243-247` borra ChatMessage, ChatUsage y Chat, no las aprobaciones (sin FK). Decidir si
    se borran; `ChatInputAudit` sobrevive a propósito

- [ ] **Correlación de logs por turno** (P2)
  - `turnId` (`chat.js:214`) solo llega a usage. Pasar `{chatId, turnId}` a `executeToolCalls` / `executeTool`
    (o un child logger por turno)

- [ ] **Métricas de latencia y errores** (P3)
  - Solo se miden tokens (`ChatUsage`). Guardar `latencyMs` por request, duración y estado por tool
    (`chat.js:565-599`), errores por tipo

- [ ] **Tool loop y subagentes fire-and-forget** (P3)
  - No sobreviven a un reinicio (las aprobaciones sí, por la DB). Evaluar una cola persistente (sin exigir Redis)
    o reanudar turnos desde la DB al arrancar

---

## Server — MQTT

Decisiones y diseño del lado dron: `docs/decisions/mqtt.md`.

- [ ] **Presencia LWT al estado del device**
  - `MqttServer` guarda `muav/<device>/status` (`server/models/mqtt/MqttServer.js:128`), pero `getPresence` solo
    lo usan los tests: un dron MQTT caído no se ve hasta que vence el timeout de posiciones

- [ ] **`_getDevice` cachea el resultado, no la promesa**
  - `MqttServer.js:202`: cada paquete que llega con la primera consulta en vuelo lanza otra. Copiar el patrón de
    `MavlinkServer`

- [ ] **Gimbal PX4 por MQTT: el NaN llega como null** (Server + dron)
  - `MqttServer.js:85` serializa con `JSON.stringify`: `param3/4 = NaN` (`vehicleCommand.js:188`) llega `null` y
    PX4 no mueve el gimbal. Reusar `stringifyKeepingNaN` (`ros/rosConnection.js:9`) si el parser del
    `mqtt_client` acepta `NaN` (Python sí, nlohmann no), o un centinela acordado

---

## Server — MAVLink

Decisiones y mediciones en SITL: `docs/decisions/mavlink.md`.

- [ ] **Enlace por device y enlaces serie (SiK)**
  - Hoy los enlaces son del SERVER (`MAVLINK_LINKS`) y el vehículo se encuentra por sysid en cualquiera. Atarlo a
    un enlace serviría para rechazar un sysid que llegue por otro
  - Serie sin implementar (`links.js:29` lo rechaza) hasta tener las radios. Con ella: `UPLOAD_STEP_MS = 1500`
    fijo (`MavlinkServer.js:61`) dará timeouts por SiK (~1-2 KB/s); SiK multipunto; `serialport` en el Dockerfile

- [ ] **Documentar que un vehículo no puede estar dado de alta por ROS 2 y MAVLink a la vez**
  - Sus comandos competirían. Agregarlo a `server/CLAUDE.md` › MAVLink Protocol

---

## Server — Arquitectura

- [ ] **Renombrar `category` → `interface`**
  - `category` (`config/devices/interfaces/`, `Device.category`) es la INTERFAZ, el conjunto de mensajes + topics +
    encoders de un stack, nunca fue una categoría de dispositivo. NO reutilizar "categoría" para el tipo de vehículo
  - Toca DB (migración de columna), API, cliente, MCP y los prompts de los agentes, que todavía nombran categorías

- [ ] **Slots de posición satelital: nombres y datum**
  - El slot `position` (GNSS) y `position_rtk` se arbitran por `priority`/`fresh_ms` (`positionReducers.js`); cada
    fuente guarda su fix en `attributes.position_gnss` / `position_rtk`, pero el SLOT sigue llamándose `position`:
    renombrarlo junto con `category` → `interface`
  - ALTITUD: se toma tal como viene. RTK y GPS pueden estar en datums distintos (elipsoide vs MSL, ~50 m en
    España): al cambiar de fuente salta la altitud en el mapa 3D y en `missionSignals`. Verificar con M210/M300
  - Calidad del fix RTK: solo frescura + lat/lon 0; `NavSatFix.status.status` no se lee y no está verificado qué
    rellena el OSDK
  - Sin histéresis al volver el RTK: la posición salta. Si molesta, exigir N mensajes seguidos. El cambio de fuente
    NO genera Event a propósito (un chip en el cliente con `positionSource` sería mejor)

- [ ] **Robofleet con los mismos topics y mensajes que ROS/MQTT → borrar `simulationFB` y `decodeFbMsg`**
  - El bridge robofleet publica sus propios nombres (`/<uav>/configureMission`…): un M300 por robofleet necesita
    una interfaz aparte (`simulationFB`). Que publique los nombres de las interfaces (`'/<device>' + name`) y los
    `.msg` tal cual, como ya espera `decodeFbViaCategory`
  - Entonces: quitar el fallback `decodeFbMsg` (`models/flatbuffer/fbDecode.js:60`, `FlatbufferServer.js:166-184`)
    y los nombres propios de `fbEncode.js`, borrar `simulationFB` y sacarlo de `dji_M300.interfaces`
  - Condición: que el warning de decoder legacy no aparezca con los devices reales. Nada verificado contra un
    device robofleet real (LoadMission por Flatbuffer, marco del `imu`, `velocity` ×0.01)

- [ ] **Editor de categorías**
  - Pierde los comentarios del archivo (`models/interfaces/interfaceStore.js:52` regenera el YAML)
  - No se puede crear una categoría desde la UI: `client/src/Navigation.jsx:100` no tiene ruta de alta y
    `EditItemView.jsx:57` haría POST a `/api/category`, que no matchea `routes/category.js:17`
  - Nunca se probó en el navegador

- [ ] **Versionado de migraciones** (`server/common/sequelize.js`)
  - En cada arranque corren 4 migraciones de esquema (`common/migrations/`, idempotentes) y una lista de
    `ALTER TABLE ... ADD COLUMN` que detecta "ya aplicada" por el error. Cada feature suma una. Al menos una tabla
    `schema_version`, o Sequelize migrations

- [ ] **Repository layer para la DB**
  - Los models usan Sequelize directo (`sequelize.models.Device.findAll(...)`). `DeviceRepository`,
    `MissionRepository` con métodos nombrados: facilita testing y desacopla del ORM

- [ ] **Esquema de error HTTP consistente**
  - Mezcla `{ error: JSON.parse(...) }`, `{ error: string }` y status codes al azar. Un formato
    `{ success: false, error: { code, message } }` y un handler global en `middlewares/errorHandler.js`

---

## Client

- [ ] **Altura relativa y distancia al home en la lista de devices**
  - `client/src/components/devices/DeviceRow.jsx:210-211` usa `localposition[2]`, que solo publica ROS: un device
    MAVLink no muestra altura. Usar `altitude - attributes.home[2]` y sumar la distancia al home
  - Sacar el cálculo de `client/src/pages/device/TelemetryGrid.jsx:64-81` a un util compartido; sin home, AMSL
    marcado como tal (como ya hace TelemetryGrid)

- [ ] **Semáforo de estado por device** (Client + Server)
  - Umbrales de batería distintos: `DeviceRow.jsx:122-136` 30/70, `TelemetryGrid.jsx:72-73` 15/30
  - Un solo nivel rojo/amarillo/verde con batería, `telemetryWatchdog` stale/missing y altura fuera de límites
    (no existe: definir de dónde sale el límite)

- [ ] **RSSI / calidad de enlace** (Client + Server)
  - Nada implementado. MAVLink trae RADIO_STATUS (SiK): decodificarlo en `server/models/msgDecode.js` y mostrarlo.
    En ROS/DJI confirmar si el hardware lo publica

- [ ] **Telemetría extendida en el detalle** (Client + Server)
  - Satélites y fix GPS, presión, IMU, latencia: no llegan del server (`msgDecode.js`, `positionReducers.js`) ni se
    muestran en `TelemetryGrid.jsx`

- [ ] **Progreso de la misión en la barra superior**
  - `client/src/components/layout/menu/MenuCenterStatus.jsx` solo muestra nombre y estado; "WP x/total" solo se ve
    en el popover (`MissionDetailPopover.jsx:61-63`)

- [ ] **Correlacionar el ACK con el command enviado** (Client + Server)
  - `client/src/pages/device/useDeviceCommands.js:42-53` atribuye cualquier ack a `lastLabelRef`; Gimbal manda
    CONFIGURE + PITCHYAW (dos acks). Usar `commandAckRaw` (`msgDecode.js:163`) contra el command enviado

- [ ] **`relativeTarget` duplicado** (Client + Server)
  - `client/src/pages/device/manualControl.js:61` y `server/models/commands/vehicleCommand.js:120` hacen la misma
    cuenta. Un golden test con los mismos fixtures en los dos lados

- [ ] **Formato markers en red y YAML: `markersbase` → `bases`** (Client + Server)
  - El storage ya está separado (`server/models/markers/*.js`), pero la forma `{markersbase, elements}` sigue en
    `server/models/markers/markers.js:65-83`, `server/controllers/websocket.js:133`,
    `client/src/pages/PlanningPage.jsx:144-190`, `client/src/pages/Scene3DEditorPage.jsx:401`,
    `server/data/*.yaml` y lo que consumen planner y MCP. Todo en un commit (contrato WS)

- [ ] **Suelo 3D que siga a la cámara**
  - `client/src/scene3d/core/MapTileGround.jsx:81-100` (7x7) y `MapVectorGround.jsx` (5x5) cargan una grilla fija
    alrededor de `scene3d.origin`: pasados ~2.6 km el dron vuela sobre el vacío

- [ ] **Consolidar la conversión geo↔XYZ**
  - Helpers inversos locales en `client/src/pages/Scene3DEditorPage.jsx:50-75` y `:619-622` → a
    `client/src/scene3d/core/convertion.js`. Revisar los `MercatorCoordinate` directos de `MapMissions3D.js` y
    `MapMarkers3D.js` (ojo: unos usan Z relativa y otros absoluta)

- [ ] **Persistir el origen 3D**
  - `client/src/store/session.js:31` arranca siempre en Sevilla y `updateScene3dOrigin` (`:105`) no persiste.
    localStorage (con try/catch) o derivarlo de la base / misión activa

- [ ] **Probar en el navegador los formularios de protocolo y System ID**
  - `DeviceIdentityEditor` (MAVLink F5) solo se verificó contra la API

---

## Tests

- [ ] **`test-definition-resolver.js` falla**
  - Subtest "nacelle … nacelle_heading_deg=240" (expected true): depende del archivo guardado del aerogenerador

- [ ] **`test-mcp-client.js` no es un test**
  - Necesita un MCP vivo en `:3001` y hace `process.exit`. Inestable conocido aparte: `test-raw-tap.js:161`
    (`ageMs >= 20` tras `sleep(20)`, el timer dispara 1–2 ms antes)

- [ ] **Tests de integración del orquestador del chat**
  - Con LLM, MCP y DB falsos: reconexión y reintento de MCP, fallo de DB en `addMessage`, timeout del turno, la
    cola de `_acquireChatLock` (`chat.js:176-202`)

---

## Flota y lado dron (fuera de este repo)

- [ ] **Alinear `px4_ros2_docker` con la versión de PX4 que espera el GCS**
  - Su README dice PX4 1.15.4 + `px4_msgs` `release/1.15`, y hay un checkout así en `~/work/px4_ros2_docker`. La
    interfaz `px4_ros2` del GCS usa topics versionados (`vehicle_status_v4`, `battery_status_v1`) que solo existen
    en `px4_msgs` `main` (PX4 1.18): con 1.15 no llega la telemetría. Versión esperada: `infra/external.md`
  - Decidir qué checkouts duplicados sobran (`px4_msgs` ×4, `psdk_ros2` ×3, `PX4-Autopilot` ×2,
    `multiUAV_system` ×2)

- [ ] **Completar `infra/external.md` con los repos `MUAV_*` del lado dron**
  - `MUAV_px4_offboard`, `MUAV_psdk_offboard`, `MUAV_GCS_gz`, `MUAV_psdk_gz` (en `px4_ros2_docker`): no están en el
    manifiesto porque ninguna interfaz del GCS los nombra; decir cuáles hacen falta para que un dron hable con el
    GCS y sumarlos a `infra/repos/ros2_px4.repos`

- [ ] **Imagen del ROS bridge: `muavgcs:ros1` vs `muavgcs:noetic`**
  - `compose.ros1.yaml` espera `muavgcs:ros1`; la imagen local es `muavgcs:noetic` (no verificado que tenga
    `/root/catkin_ws` con `aerialcore_gui`). Retaggearla o poner `ROS1_IMAGE` en `infra/local.env`

- [ ] **Construir `infra/docker/ardupilot-sitl`** (reconstruido de las capas de `muav-arducopter-sitl:4.7.2`, nunca
  compilado así) y probar la variante `rover`

- [ ] **Pasar `vtol_1` a `standard_vtol`** (quedó como `x500`) y cambiar el launch del relay del dron a
  `/<device>/slow/<topic completo>`

- [ ] **Habilitar las capacidades opcionales en las unidades que tienen el hardware**
  - `rtk` en los M210/M300 con módulo; `gimbal` / `camera` / `camera_stream` / `lidar_3d` en los x500 que los
    tengan. Sin `camera` un x500 no entra en los tipos de misión de inspección

- [ ] **Verificar los datos físicos de los device models**
  - `config/devices/device_models/_dji_rotorcraft.yaml` (y `dji_M600`): masa, rotores, coeficientes, batería,
    velocidades y sensores copiados TAL CUAL del viejo `devices.yaml` (todos 7.2 kg, rotor 0.226 m…). Los usa el
    modelo de energía de Smart-Flies: un dato falso planifica mal sin avisar. Contrastar con las fichas de DJI
  - Mientras tanto los devices están en modelos genéricos: el planner no gana nada hasta cargar los reales

- [ ] **Verificar la interfaz `sjtu` contra el simulador**
  - sjtu_drone (github.com/NovoG93/sjtu_drone), copia en `/home/grvc/work/sjtu_drone`; arranque
    `tmuxinator start -p /home/grvc/work/sjtu_drone/tmuxinator.yaml` (ROS 2 Iron + rosbridge en el contenedor)
  - Según su README `~/gps/vel` es `TwistStamped` (la interfaz dice `Vector3Stamped`); el estado de vuelo es
    `~/state` (`Int8`) y la interfaz usa `/joint_states`; namespace `/simple_drone` vs `/drone`; messageType sin
    `/msg/`. Expone `~/takeoff`, `~/land`, `~/cmd_vel`, `~/posctrl` y la interfaz no declara ninguno

- [ ] **PX4: publicar la actitud del gimbal**
  - `/fmu/out/gimbal_device_attitude_status` en el `dds_topics.yaml` del dron y `pub/gimbal` en el `mqtt_client`
    (el GCS ya lo suscribe, `px4_ros2.yaml:25-27`). Sin eso, el eje que falta cae a 0

- [ ] **`mqtt_client`: bridge de servicios y config**
  - Spike F3a (`GenericClient`: CDR o struct, `.srv`) y `ServiceBridge` F3b; plan B `rclpy`. Sin esto ningún
    command MQTT llega al dron (ver `docs/decisions/mqtt.md`)
  - Config con las keys snake_case (`srv/command_mission`, `pub/imu`…) y los tipos ROS 1 normalizados a `srv/`
  - Seguridad: ACL en mosquitto (solo el GCS escribe `srv/+/request`), descarte por `ts` (NTP/chrony o expiry de
    MQTT 5), LWT. Un request de `command_mission` arma un dron

---

## Pruebas en SITL / hardware

- [ ] **PX4 SITL — gimbal**: yaw absoluto (YAW_LOCK) con `MNT_DO_STAB=0`; signos del cuaternión y qué yaw reporta
  según los flags (`msgDecode.js:231-257`); gimbal manager v2 por MAVLink (nunca volado)
- [ ] **PX4 SITL — pausa/resume** (`pause_resume.mjs px4`; en ArduCopter está verificado)
- [ ] **PX4 SITL — `stay` (hold) en ala fija**
- [ ] **MQTT de punta a punta en SITL**: QoS `best_effort` de `px4_msgs`, respuestas con tipos anidados, misión
  cargada y ejecutada
- [ ] **OSDK (M210/M300) — gimbal absoluto**: pitch -30 dos veces (el segundo no debe mover); yaw de referencia
  heading del dron o norte
- [ ] **`wp_reached` en devices `muav_state_machine` (CATEC/DJI)**: desde que `getByDeviceId` es sincrónico
  completa tareas en todos los protocolos, nunca corrió con ellos: vigilarlo en la próxima misión real
- [ ] **Planner externo de `planning.js` (`/get_plan`)**: en qué referencia de altura manda (sin marca se toma
  relativa)

---

## BAJO — Vehículos sin unidad que los use todavía

- [ ] **ArduPlane / ArduRover**: sus modos en `vehicleStates.js` (hoy caen en `OTHER`)
- [ ] **VTOL por ROS**: `vtolState` y `vtol_transition` por `px4_ros2` / `px4_mavros` (su encoder descarta la
  acción con un warning; solo MAVLink la manda)
- [ ] **USV / UGV**: `pos[2]` sin sentido, sin takeoff/land, `flightPhase` de superficie
- [ ] **Cooperación UAV↔USV** sobre el DAG de tareas (base móvil, aterrizar sobre un vehículo)

---

## BAJO — Limpieza

- [ ] **Borrar `localPlanning`**: código muerto con `uav_15` y home hardcodeados (`server/models/planning.js:213`,
  `server/controllers/planning.js:51`, sin llamadores)
- [ ] **Recortar `docs/decisions/mavlink.md` y `mqtt.md`**: las secciones de diseño de F1–F6 (MAVLink) y "Contrato
  MQTT" / "Pieza 3" (MQTT) repiten `server/CLAUDE.md`. Conservar decisiones, mediciones, hallazgos, el diseño del
  lado dron y las anclas que cita el código (`› Resultados F0`, `› F0`, `› F4`)
