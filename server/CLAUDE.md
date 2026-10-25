# Server CLAUDE.md

## Directory Structure

```
server/
├── server.js                    # Main entry point (Express app initialization)
├── package.json                 # Dependencies (Express, Sequelize, ROSLIB, LLM SDKs, etc.)
├── Dockerfile                   # Docker container configuration
├── .env                         # Environment configuration (PORT, DB, ROS, LLM, etc.)
├── WebsocketManager.js          # Client WebSocket transport (raw, no business logic)
├── WebsocketInboundRouter.js    # INBOUND_MAP: routes client WS messages by `type` to controllers
├── routes/                      # Express route definitions (API endpoints)
├── controllers/                 # Request handlers (business logic delegation)
├── models/                      # Core business logic, one subfolder per domain
│   ├── ros/                     # ROS1/ROS2 integration (facade + transport primitives, see below)
│   ├── flatbuffer/              # FlatBuffer device protocol (FlatbufferServer, fbEncode/fbDecode)
│   ├── mqtt/                    # MQTT device protocol (MqttServer, mqttTopics)
│   ├── mavlink/                 # MAVLink device protocol (MavlinkServer, links, mavlinkMessages)
│   ├── gateways/                # One command gateway per device protocol (see Device Communication Protocols)
│   ├── chat/                    # LLM chat orchestrator, provider handlers, agent profiles (see below)
│   ├── commands/                # Device commands: sendCommand, command schema, encoder registry, PX4 VehicleCommand builder
│   ├── mission/                 # Mission/route state machines, XYZ<->geodetic conversion, encoders
│   ├── collision/                # UAV-to-UAV and obstacle collision detection/validation
│   ├── markers/                 # Inspection element catalog: bases, groups, items, assignments
│   └── positions/               # Position cache, history sampler, broadcast batcher
├── schemas/                     # Data schemas
│   ├── database/                # Sequelize ORM models (see Database Models below)
│   └── zod/                     # Zod validation schemas
├── subscribers/                 # EventBus subscribers (outbound WS adapters)
├── common/                      # Shared utilities (eventBus, logger, sequelize, FTP/SFTP, geo)
├── middlewares/                 # Express middlewares
├── config/                      # Configuration files (config.js is the env-var SSOT)
├── scripts/                     # One-off maintenance/migration scripts
├── fbmsglib/                    # FlatBuffer message library (generated + schema)
├── test/                        # Node test-runner suites, API fixtures, E2E scripts
├── views/                       # Server-rendered views
├── resources/                   # Static resources
├── logs/                        # Log file output
└── data/                        # Runtime data storage (SQLite DB, element-type assets, etc.)
```

## Architecture Overview

### Communication Architecture

**Three-Layer WebSocket System:**

1. **Client WebSocket** (`/api/socket`): Server → Browser clients

   - Managed by `WebsocketManager.js`in server and `SocketController.jsx`in client
   - Broadcasts position updates (2s interval), server state (10s interval)
   - JSON message format with type-based Redux dispatch
   - Auto-reconnect with 60s retry interval
   - Camera frames are the exception: they're raw **binary** WS messages, not JSON
     (see `subscribers/cameraStreamSubscriber.js`) — pushed the moment ROS publishes
     a new frame, not on the polling interval. Wire format: `[1 byte msgType][1 byte
     len(deviceId)][deviceId UTF-8][JPEG bytes]`. The client (`SocketCameraCanvas.jsx`)
     reads them straight off `window.websocket`, outside Redux.

2. **ROS WebSocket** (`ws://127.0.0.1:9090`): ROS Bridge → Server

   - Uses ROSLIB via the `models/ros/` module (facade `models/ros/ros.js`)
   - Auto-subscribes to device topics based on `devices_msg.yaml` configuration
   - Decodes ROS messages via `models/msgDecode.js` into internal position/status format
   - Auto-reconnects every 30s on disconnect (see `models/ros/rosConnection.js`)

3. **Device WebSocket** (FlatBuffer protocol): Server → UAV Fleet
   - Alternative to ROS for direct fleet communication
   - Binary FlatBuffer encoding via `models/flatbuffer/FlatbufferServer.js` (started from `server.js` via `initFlatbufferServer(8082)`; encode/decode live in `fbEncode.js`/`fbDecode.js`)
   - Used for non-ROS devices or optimized network communication

**EventBus Pattern (outbound):**
All state changes flow through a central EventBus (`common/eventBus.js`):

```
Business Logic / scheduler → eventBus.emitSafe() → WebSocketSubscriber
(OUTBOUND_MAP: event → payload) → wsController.sendMessage() → Clients
```

The periodic scheduler (`websocketController.updateserver`, 10s) is a plain event
producer for `DEVICE_UPDATED`/`SERVER_UPDATED` — it no longer touches the socket.
The `WebSocketSubscriber` is the single outbound adapter for JSON — its
`OUTBOUND_MAP` transforms are pure `(data) => payload`, no binary payloads allowed
there by design.

Positions are NOT polled from a full snapshot, but they ARE batched — one
`POSITION_UPDATED` broadcast per tick with every device that changed since the
last one, not a message per device and not the whole cache:

```
positionsController.updatePosition() → positionBroadcastBatcher.stage(position)  (every ROS/FlatBuffer message, up to 50 Hz/device)
positionBroadcastBatcher timer (WS_POSITIONS_INTERVAL_MS) → eventBus.emitSafe(POSITION_UPDATED, [...changedDevices])
→ WebSocketSubscriber → wsController.sendMessage() → Clients
```

`positionBroadcastBatcher` (`models/positions/positionBroadcastBatcher.js`,
started/stopped from `server.js` like `positionHistorySampler`) holds a `Map`
keyed by `deviceId`: `stage()` just overwrites that device's entry with its
current merged state (from `positionsModel`, not the raw ROS message — a single
message may carry only a subset of fields, e.g. battery-only). A fixed timer at
`WS_POSITIONS_INTERVAL_MS` (default 500ms → 2 Hz) drains the whole map into one
`POSITION_UPDATED` emit and clears it. This buffer does double duty: it caps each
device to at most one broadcast per tick (dead-band by time, same idea as
`positionHistorySampler`) AND groups whichever devices changed in that window into
a single WS message/React re-render, instead of one per device. The client's
`updatePositions` reducer upserts by `deviceId`, so a partial batch (not all
devices) is fine. A client that just connected gets the full snapshot once via
`WelcomeMessage`, which bypasses this event entirely (sent directly, not through
`OUTBOUND_MAP`).

Camera follows the same per-message shape, binary instead of JSON:

```
positionsController.updateCamera() → eventBus.emitSafe(CAMERA_RECEIVED) → CameraStreamSubscriber
→ encodeCameraFrame() → wsController.sendBinary() → Clients
```

`CAMERA_RECEIVED` fires per-message, straight from the ROS ingestion callback
(`models/ros/ros.js`'s `onCamera`) — same pattern as `POSITION_RECEIVED`, just for
cámara, and unbatched — every frame goes out immediately, no `positionBroadcastBatcher` equivalent. `CameraStreamSubscriber`
(`subscribers/cameraStreamSubscriber.js`) is the only place that knows the wire
format; `websocketController.setupWelcomeMessage()` reuses its `encodeCameraFrame()`
to unicast whatever's cached to a client that just connected, so it doesn't wait for
the next ROS frame to see something.

**Inbound routing:**
Client messages are delegated raw by `WebsocketManager` to `WebsocketInboundRouter`
(`INBOUND_MAP: type → handler`), which dispatches the command directly (e.g.
`CHAT_USER_MESSAGE` → `chatController.processMessage`). The transport knows no
business types. Any reply back to the client goes out via the EventBus → subscriber.

**Targeted outbound + raw inspector:** each WS connection gets an ephemeral `id`
(`WebsocketManager.clients`, removed on `close`; `sendTo(id, msg)`). An `OUTBOUND_MAP`
transform may return `toClient(clientId, payload)` instead of a plain payload to send to
that ONE client rather than broadcast (`websocketSubscriber.js`, `Targeted`). First user:
the raw message inspector (`models/inspect/rawTap.js`). A client sends `raw:subscribe
{deviceId, slot}` / `raw:unsubscribe`; ros.js and MqttServer call `rawTap.publish()` with
the message BEFORE the decode; `rawTap` emits `RAW_MESSAGE` only when somebody is
subscribed, at most once per `WS_RAW_INSPECT_INTERVAL_MS` per client and slot, and the client
receives `{ raw: {deviceId, slot, transport, ts, data} }`. Inside a window the first message goes
out at once and the NEWEST one goes out when it closes; the ones in between are dropped (a state
change that arrives right after another message of its slot must not be lost). Every
`raw:subscribe` is answered with `{ rawStatus: {deviceId, slot, ok, reason?} }` (`RAW_STATUS`,
same targeted path; reasons in `RAW_REJECT`: `invalid`, `not_inspectable`, `too_many` — a client may
hold 8 subscriptions). Subscriptions die with the socket (`onClientClose`). `rawTap` also keeps the
NEWEST message of every slot (camera excluded), subscribed or not, and the router replays it right
after an accepted `RAW_STATUS` (`replayLast`) as `{ ..., cached: true, ageMs }`: without it an
event-driven slot (`vehicle_command_ack`, `threat`) showed nothing until the next event. `ts` is the
ARRIVAL time; `ageMs` is computed on the server so the client clock does not matter. A lazy `raw`
(robofleet, MAVLink) stays UNRESOLVED in the cache, holding its buffer, until a client receives it
(one buffer per device and slot); it is then resolved once and the thunk dropped.
`test/test-raw-inspector-ws.js` runs the whole path with real `ws` clients. The `camera`
slot is not inspectable. FlatBuffer is tapped too (`FlatbufferServer._onMessage`): there the
`slot` is looked up in the device's category (`slotOfTopic`: the `subscribers` key whose `name` is
the topic, same `'/<device>' + name` rule as ROS; the `simulationFB` category declares what
fbDecode understands today). A topic the category does not declare keeps its own name without the
device prefix. **Decoding goes through the category too** (`decodeFbViaCategory`): the message is
unpacked into the ROS message it carries (`fbMsgAsRos`: `.msg` field names, no robofleet metadata)
and handed to the same `msgDecode` registry as rosbridge/MQTT, so ROS is the reference and a
robofleet device decodes like a ROS one. Only a topic the category does not declare (a bridge not
renamed yet), a type mismatch or the camera slot falls back to the old per-type `decodeFbMsg`;
that fallback is meant to disappear once the bridge publishes the `devices_msg` names. The `.msg`
name of each generated field is read from the `.fbs` (`rosFieldName`): flatc camelCases
`frame_id` but keeps DJI's `curWaypointIndex`, so it cannot be guessed. Constants declared inside
a table (`HEALT_WORKING` in ObstacleInfo, which flatc makes a real field) are dropped. Never edit a
`.fbs` to fix that: removing a field renumbers the vtable and breaks the bridge. `raw`
is a function that `rawTap` runs lazily (`unpackFbMsg`, `models/flatbuffer/fbUnpack.js`): it
resolves the class from the ROS type (`pkg/Name` → `fb[pkg][Name]`, no table to maintain; a type
the lib does not have comes back as hex) and unpacks it with the object API of `fbmsglib`.

Key events: `MISSION_PLAN_SHOWN`, `MISSION_UPDATED`, `TASK_UPDATED` (WS `taskUpdated`; also consumed by `missionWpTracking` and `taskScheduler`), `POSITION_UPDATED`, `CAMERA_RECEIVED`, `DEVICE_UPDATED`, `CHAT_CREATED`

### Database Models

Location: `server/schemas/database/`

**Existing Models** (`schemas/database/index.js` is the barrel — register new ones there):

| Model            | Purpose                                                |
| ---------------- | ------------------------------------------------------- |
| User              | Auth/user registry                                     |
| Device            | UAV registry                                           |
| Mission           | Mission tracking (`request` = the originating mission request) |
| MissionTask       | One node of the mission's task graph (`taskKey`, `dependsOn`, `action`) |
| MissionPlan       | LLM/MIP-planner-generated plan, pre-execution           |
| PositionHistory   | Sampled position history for the map's time slider      |
| File              | Downloaded mission artifacts (logs, images), `taskId` → MissionTask |
| Event             | Device/mission event log                                |
| Geofence          | Geofence polygons                                       |
| Chat              | LLM chat session                                        |
| ChatMessage       | Individual chat turn (user/assistant/tool)              |
| ChatUsage         | Per-request/turn/chat LLM token usage                   |
| ElementType       | Inspection element type catalog (icon + 3D model)       |
| ElementGroup      | Group of inspection items (e.g. one wind turbine)       |
| ElementItem       | Individual inspection viewpoint within a group          |
| Base              | UAV base/landing-pad location                           |
| Assignment        | UAV-to-target assignment produced by planning           |

**Adding New Models:**

1. Create `server/schemas/database/newModel.model.js`
2. Register in `server/schemas/database/index.js`
3. Create business logic in `server/models/newModel.js`

### State Management

**Server-side State:**

- Device registry: SQLite database with periodic health checks (30s timeout for OFFLINE status)
- Mission tracking: one XState actor per TASK (`models/mission/missionSM.js` spawns one `missionExecutionSM.js` machine per `taskId`, keyed in `listSM`), dispatched by `taskScheduler` — see Mission Planning System
- Position updates: In-memory cache with EventBus broadcast

**Redux Data Flow:**

```
WebSocket → SocketController.onmessage → dispatch(action) →
Reducer updates state → useSelector triggers re-render
```

### Mission Planning System (task-dependency graph)

A mission is a DAG of **tasks**. Each task runs on ONE device, as waypoints, once the
tasks it depends on have completed (e.g. USV navigates → UAV takes off and surveys →
USV navigates back). `models/mission/taskGraph.js` is the SSOT for the format and its
validation (pure functions, tested in `test/test-task-graph.js`).

> Naming: "task" means a node of this graph. What the external system (ExtApp) or the
> planning UI sends to ask for a mission is a **mission request** (`Mission.request`,
> `requestMission`, `POST /missions/requests`) — it used to be called "task".

**Mission format v4** (what planners/the editor send and what is stored in `MissionPlan`):

```javascript
{
  version: '4',
  name, description, global_origin,   // mission-level, unchanged
  tasks: [{
    task_id: 'T1',            // unique within the mission; what depends_on refers to
    device: 'usv_1',          // device name (was route.uav)
    action: 'NAVIGATE',       // SEMANTIC only (UI, logs, LLM): every task flies as wp[] + configure_mission
    depends_on: ['T0'],       // task_ids; satisfied when they reach COMPLETED (last waypoint)
    params: {                 // was route.attributes; values + ranges from config/devices/mission_schema.yaml (SSOT, which calls them `params`)
      max_vel: number,        // m/s
      idle_vel: number,       // m/s
      mode_yaw: 0-5,          // HEADING_MODE_* (0=Auto,1=Fixed,2=RC,3=Waypoint custom,4=POI,5=Gimbal yaw)
      mode_gimbal: 0-1,       // GIMBAL_MODE_* (0=Free,1=Waypoint)
      mode_trace: 0-3,        // FLIGHT_PATH_MODE_* (0=Curve,1=Curve+stop,2=Straight+stop,3=Coordinate turn)
      mode_landing: 0-4       // MISSION_FINISHED_* (0=No action,1=Home,2=Land,3=First WP,4=Infinite)
    },
    uav_type, name,           // optional
    altitude_ref: 'relative', // what a GEODETIC wp.pos[2] means (ALTITUDE_REF, taskGraph.js): 'relative' (absent:
                              // height over the takeoff; the editor and convertMissionXYZToLatLong, whose
                              // origin has alt 0) | 'amsl' (nothing produces it today). A typo is a 400
    wp: [{
      type: string,           // "takeoff" | "inspection" | ... — first wp is usually "takeoff"
      pos: [x, y, z],         // LOCAL ENU meters, NOT lat/lon — z is altitude AGL
      yaw: -180 to 180,       // 0=North, 90=East, -90=West (optional)
      gimbal: number,         // Pitch angle in degrees (optional)
      speed: number,          // m/s (optional)
      notes: string,          // human-readable waypoint label (optional)
      action: {}              // Custom per-waypoint actions (optional)
    }]
  }]
}
```

- **Legacy `route[]` is still accepted everywhere** through `normalizeMission`: route *i*
  becomes task `T{i+1}` with no dependencies, `attributes` → `params`, `uav` → `device`,
  action `ROUTE`. The task_id comes from the array index, not `route.id` (planners emit
  duplicate ids). The planners (`mip_planner`, `planner.md`/MCP `MissionSchemaXYZ`) still
  emit `route[]`, so **the automatic flow gets no dependencies until they emit `tasks[]`**.
- A task carrying `attributes` but no `params` is rejected: it would silently fly on defaults.
- `validateTaskGraph` rejects: duplicate/missing ids, unknown or self dependencies, cycles,
  and **two tasks of the same device not ordered by the graph**. That last rule is the
  invariant the whole execution relies on: **a device has at most one active task**, which
  is what lets device-only ROS signals resolve to a task. Errors are returned all at once;
  over HTTP they are a `400 { error, errors }`.

**Persistence:** `Mission` (1) → `MissionTask` (N, `taskKey` unique per mission, `dependsOn`,
`action`, `status`, `currentWp`/`totalWp`) → `File.taskId`. `MissionPlan.missionData` is
stored as received (`route[]` tagged `'3'`, `tasks[]` `'4'`), and `createMissionPlan` rejects
an invalid graph on every path, `POST /missions/plans` included; server code always reads it
through `normalizeMission`. `POST /missions/convert/xyz-to-geodetic` converts the `wp` of
`tasks[]` or `route[]` and keeps every other field. The schema migration (`MissionRoute`→`MissionTask`,
`File.routeId`→`taskId`, `Mission.task`→`request`) lives in
`common/migrations/missionTaskGraph.js` and runs at startup **before** `sequelize.sync()` —
otherwise sync creates an empty `MissionTask` next to the old table and the rename can
never happen. It is idempotent and uses SQLite's native `RENAME COLUMN` (Sequelize's
SQLite `renameColumn` rebuilds the table and can drop FK `REFERENCES`).

**Task status** (`TASK_STATUS`, `config/status.js`): `init` (waiting for dependencies) →
`loaded` → `commanded` → `running` → `complete` (last waypoint: satisfies dependents) →
`end` (closed, files handled); plus `cancelled`, `error`, and `skipped` (never ran: a
dependency failed). Sets: `TASK_ACTIVE_STATUS` (holds its device), `TASK_TERMINAL_STATUS`,
`TASK_FAILED_STATUS`. `skipped` ≠ `error`: a skipped task never flew, collapsing them
loses why the mission degraded. A mission closes when every task is terminal: `finish`, `finish_errors`
(some failed/skipped) or `error` (none completed). Only an **alive** mission is closed
automatically, so a mission already `cancelled`/`error` keeps its status and message.

Coordinates are XYZ local ENU end-to-end through planning (see `mip_planner/CLAUDE.md`
and `models/mission/coordinateConverter.js`). Conversion to/from geodetic
(`convertMissionXYZToLatLong`, `geodeticToENU`/`ENUToGeodetic`) happens only at the
edges — when talking to ROS/PX4 (`missionEncodeConfig.js` maps `pos[0..2]` straight
to `latitude/longitude/altitude` fields expected by the firmware bridge, so double-check
which coordinate frame a given consumer expects) or when rendering on the geo map.

**Mission Execution Flow:**

Both flows create **every** task (`init`) first and then let the scheduler dispatch them.
Order matters: marking a task `error` skips its dependents, so they must already exist.
A task whose device doesn't exist is still created (`deviceId` null) and then failed, so
its dependents are skipped instead of waiting forever.

- **Automatic** (ExtApp → `POST /missions/requests` → planner → `initMission`): validate the
  graph **before** persisting anything (invalid plan → mission `error`, no `MissionPlan`
  left behind) → create plan → mission `running` → create tasks → `taskScheduler.dispatchReady`.
- **Manual** (editor → `POST /missions/load`, then `POST /missions/command`): `load`
  validates, creates all tasks and loads only the **roots** (no `depends_on`); a root that
  fails to load goes to `error`. `command` commands the `loaded` roots, attaches a state
  machine already in `RunningMission` to each (`AttachRunning`, so manual roots also
  download files and free their device), sets the mission `running` and hands over to the
  scheduler. A repeated `command` on a mission that is not `init` is a no-op warning.
- **After a restart** (`failInterruptedMissions`): alive missions go to `error` first (so
  closing their tasks can't re-close them), then `init` tasks → `skipped`, active → `error`.
  No resume.

**Scheduler** (`models/mission/taskScheduler.js`, tested in `test/test-task-scheduler.js`):
- Event-driven: `TASK_UPDATED` (any task reaching a terminal status) and the internal
  `TASK_DEVICE_RELEASED` (a task's state machine reached `END`).
- A task is dispatched when its mission is `running` (the gate that keeps manual missions
  still until commanded), every `depends_on` is `complete`/`end`, **and its device is free**:
  no state machine in flight and no other `loaded`/`commanded`/`running` task in ANY
  mission. Dependencies are satisfied at the last waypoint, but the device may still be
  running its end-of-mission action (RTL/land) until its machine ends — this is what stops
  two consecutive tasks of one device from overlapping.
- A failed/cancelled/skipped task skips its whole downstream (transitively, `init` only).
- Every read-decide-dispatch runs in a **per-mission serialized queue**: that, not a DB
  claim, is what prevents a child from being dispatched twice when two parents complete
  together. Single-process assumption.

**State machines** (`missionSM.js` registry + `missionExecutionSM.js` definition):
- **One actor per task**, keyed by `taskId` in `listSM`. Creating an actor immediately
  sends `configure_mission` to the device, so only the scheduler and the flows create them.
- The device is loaded with a **one-task mission** (`missionForTask`, dependencies
  stripped). The `LoadMission` encoder (`missionTaskForDevice`, `models/commands/commandRegistry.js`) accepts any mission shape (incl. the bare `route[]` the
  MCP `load_mission_to_uav` tool posts) and loads the device's single task; a payload with
  several tasks for that device is rejected rather than guessed.
- ROS end-of-mission signals carry only the device: `deviceFinishedMission` /
  `deviceSyncedFiles` resolve them through the actor in `RunningMission` /
  `UAVDownloadFiles`, **not the DB** — wpTracking may already have marked the task `complete`.
- A load/command failure (`resetUAV`) marks the task `error` with the reason; left in
  `init` it would be re-dispatched forever.
- **Files are downloaded once per device**, by its last open task: `DecideDownload` asks
  `shouldDownloadFiles` when the task *finishes* (not when dispatched), so if a later task
  of the device got skipped meanwhile, this one downloads. Earlier tasks go through
  `FinishWithoutDownload`. The download window is the mission's `initTime` → now. A device whose
  category offers no `CameraFileDownload` (MAVLink, `sjtu`…) never downloads: without that rule its
  machine waited in `UAVDownloadFiles` for a `downloadFilesGCS` that never comes (no `onDone`) and the
  scheduler kept the device busy until a restart.
- Full XState definition: states `LoadMission`, `Commadmission`, `RunningMission`,
  `DecideDownload`, `UAVDownloadFiles`, `FinishWithoutDownload`, `DownloadFilesGCS`,
  `resetUAV`, `return2home`, `stopMission`, `END` — not a simple linear pipeline.

**Endpoints:** `GET /missions/tasks` (tasks, filters `id`/`missionId`/`deviceId`/`status`),
`POST /missions/requests`, `/load`, `/command`. Deprecated aliases kept for external
systems: `GET /missions/routes` (MCP `resources.ts`) and `POST /missions/sendTask`
(ExtApp) — remove them once those callers migrate.

**Known gaps:** planners still emit `route[]` (no dependencies in the automatic flow);
`validateMissionCollission` treats tasks ordered by the graph as concurrent (possible
false inter-UAV conflicts); `MissionTask.initTime` is the creation time, not the start
time, so per-task report windows (events, flown track) of two tasks on one device overlap.

**Command Execution:**

- Two channels: ROS services OR FlatBuffer commands
- 5-second timeout with automatic error callbacks
- Command contract: `config/devices/command_schema.yaml` (same pattern as `mission_schema.yaml`) says what each command means — params with unit/min/max/default/required, label, group, `severity`, `confirm`, `hidden`. WHICH endpoint of a device implements a command, and how its message is built, is a registry like `msgDecode`'s: `models/commands/commandEncoders.js` (`register(CommandType, { key?, msgType? }, encoder)`; specific msgTypes such as `px4_msgs/msg/VehicleCommand` match by msgType, generic ones such as `std_srvs/SetBool` by the endpoint key) filled in `models/commands/commandRegistry.js`. Availability is DERIVED from the endpoints the category already declares — no per-category list. A command matching two endpoints of one category is a boot problem until `commands.<Type>.via` (e.g. `publishers.vehicle_command`) picks one; `test/api/json/category_commands.json` is the golden list per category. A category may narrow a command's params in its `commands:` block of `devices_msg.yaml` (min/max/step/default/options only, never widen). `models/commands/commandSchema.js` validates both at boot and resolves them; `GET /api/commands/types` returns each command resolved for the device, and `sendCommand` validates `attributes` against that SAME resolved command (out of range / wrong type / unknown / missing required → `{ state: 'error', msg }`, never clamped; defaults filled), so handlers and builders get clean values. `freeform: true` skips it (LoadMission and UploadMission: their payload is a mission, validated by taskGraph)
- Command types: `CommandType` in `config/commandTypes.js` (PascalCase: `LoadMission`, `CommandMission`, `CameraFileDownload`, …). Not the same thing as the endpoint key that implements it on a device (`configure_mission`, `command_mission`: keys of `devices_msg.yaml`, snake_case, enumerated by `SubscriberKey`/`ServiceKey`/`PublisherKey` in `config/deviceMsgCatalog.js`). A key names a slot, not the ROS name (that is `name:`), but it IS the MQTT topic segment (`muav/<device>/pub/<slot>`, `muav/<device>/srv/<key>`), so renaming one means updating the drone's `mqtt_client` config. The client mirrors it in `client/src/common/util/commandTypes.js`; `test/test-command-types-mirror.js` fails if they drift

### ROS Integration

**Device Configuration** (`server/config/devices/interfaces/<category>.yaml`, one file per
category). A "category" is an INTERFACE — how the GCS talks to a device (message set + topics +
encoders of one software stack), not a kind of vehicle (plan: `VEHICLE_PLATFORMS_PLAN.md`). The
name `devices_msg` survives in code and comments for this catalog (it was one `devices_msg.yaml`).

```yaml
# config/devices/interfaces/dji_M210_noetic.yaml — the file name IS the key
subscribers:
  position:
    name: '/dji_osdk_ros/gps_position'
    messageType: 'sensor_msgs/NavSatFix'
  battery:
    name: '/dji_osdk_ros/battery_state'
    messageType: 'sensor_msgs/BatteryState'
services:
  configure_mission:
    name: '/dji_control/configure_mission'
    messageType: 'aerialcore_common/ConfigMission'
```

Every `*.yaml` of the folder is an interface (no index to register it in; other extensions are
ignored, a file name that is not a key `^[A-Za-z0-9_]+$` is reported). Each one is validated by ONE
validator (`models/interfaces/interfaceCatalog.js` + the strict zod schema `schemas/zod/interface.js`)
that the boot, the API and `test/test-interfaces.js` share: an unknown key (`nmae:`), a param of the
wrong type, a ROS `name` without `/` or with spaces, a missing `messageType`, a MAVLink entry with a
`name` or an interface mixing formats are errors. An invalid file is NOT loaded (`logger.error` with
all its problems; the others keep working; `LOG_LEVEL=debug` throws) and its devices hit the boot
error "category does not exist". A new param a decoder reads must be added to the schema, or every
file using it is refused. Every block names its type with `messageType`; the old
`serviceType`/`actionType` are schema errors. (`serviceType`/`actionType` survive only as the
rosbridge argument names, inside `rosServices`/`rosAction`.)

Every concrete interface says what it can drive: the `firmwares` it works with correctly
(`FIRMWARE`: dji, px4, ardupilot, nav2, custom — one for a firmware-specific bridge such as DJI
OSDK, uXRCE-DDS or a MAVLink dialect, several for one that only translates: mavros is MAVLink in
ROS, so a mavros interface lists px4 AND ardupilot) and the `vehicleTypes` it can serve
(`VEHICLE_TYPE`: multirotor, fixed_wing, vtol, flapping_wing, usv, ugv), both in `config/status.js`.
Which firmware a vehicle actually RUNS is a fact of its device model / device (plan F2), not of
the interface; `control` (autopilot |
companion) is informative, nothing branches on it. A variant is a file with `extends: <parent>`:
resolved in two passes (file order does not matter), block entries merged one by one, field by
field (`slot: null` removes an inherited one), everything else replaced. An unknown parent, a cycle
or an invalid parent leave the child out, saying why. `abstract: true` = a base only to extend: not
listed, refused for a device (`deviceProblem`). `mission:` pins the mission_schema profile when the
one derived from configure_mission is not right (it replaced the per-category list in
`missionSymbols.js`). `GET /api/category/:category` returns the RESOLVED category; the editor loads
`GET /api/category/:category/raw` (the file as written) so saving does not copy every inherited entry
into the file. `GET /api/category?details=true` → `[{name, root, firmware, vehicleTypes, control}]`,
each `root` (top of its `extends` chain) before its variants. Names in `config/planning/config.yaml`
are checked at boot (`unknownPlanningCategories`, a warning).

**Device models** (`config/devices/device_models/<key>.yaml`, `models/deviceModels/`): WHAT a vehicle
is, apart from its interface (HOW it is talked to). A device = `deviceModel` + `category`
(`Device.deviceModel`). Same conventions as the interfaces: one file per model, file name = key,
strict zod schema (`schemas/zod/deviceModel.js`), `extends` (objects merged key by key) and
`abstract` (`_dji_rotorcraft` holds what the DJI models share). A model declares `vehicleType`
(required), the **`interfaces` a device of it can use, written out** (required, may be `[]`: the
form offers exactly these, so x500 never sees `catec`, which is mavros with CATEC's own topics), the
`firmware` it runs (a list; NOT declared = not known), `label`, `icon` / `model3d` (client keys), and
what a planner needs: `physics` / `battery` (field names are Smart-Flies-MUAVPlanner's), `speeds`,
`limits.min_turn_radius`, `obstacle_sensor`. Every vehicle type has a `generic_<type>` model (boot
problem otherwise), listing only the GENERIC interfaces of its type. The hand-written list is checked
at boot by the compatibility rule: `model.vehicleType ∈ interface.vehicleTypes` ∧ (`model.firmware`
undeclared ∨ it meets `interface.firmwares`) (`incompatibility`, pure); a model listing an unknown or
incompatible interface is NOT loaded. A device pair is valid when the model lists the interface
(`modelInterfaceProblem`). An interface's `defaultModel` (else the generic of its first vehicle type;
it must list the interface) is what a device gets when none is given: `POST /api/devices` fills it
(MCP/ExtApp create devices without one), and at boot a device with a null `deviceModel` (the column
was added empty) gets it once (`DevicesModel.#checkDeviceModel`, logged); a stored pair that no longer
fits is a warning, never a stop. `POST`/`PUT` refuse an unknown model (400, path `['deviceModel']`) or
an interface the model does not list (path `['category']`). `GET /api/device-models[?category=]`
(grouped by vehicle type, generic last; with `category`, the models that list it, default first),
`GET /api/device-models/:model`; `GET /api/category?deviceModel=` → the interfaces that model lists
(with `details=true` each carries its `messageFormat`, which decides the protocols offered). `getAll` adds `platform: {vehicleType, label, icon,
model3d}` (the generic of its type fills a missing icon/model3d): the client draws a device by it,
never by its category name. MAVLink: the HEARTBEAT's `MAV_TYPE` (`vehicleTypeOfMavType`) is compared
with the model when a vehicle comes online; a mismatch is a warning once, never a refusal. The
physics are UNVERIFIED copies of the old `devices.yaml` (TODO).

**Capabilities** (`config/devices/capabilities.yaml`, `models/capabilities/capabilityCatalog.js`): WHAT a
vehicle can do, and which COMMANDS and telemetry SLOTS implement each one (`autonomous_mission`,
`flight_control`, `navigation`, `file_download`, `gimbal`, `camera_stream`, `rtk`, `obstacle_sensing`,
`threat_detection`, `humidity_sensor`); a command or slot belongs to at most one, and what none owns
(SaveHome, position, battery…) every device has. The interfaces are untouched: they declare topics, the
catalog says what each one is for. A model declares `capabilities: { name: standard | optional }` (`{}` =
telemetry only, `'*'` = everything its interface implements, the generics); a unit switches them in
`attributes.capabilities` (`{ rtk: true }` enables an optional one, `{ gimbal: false }` disables a
standard one; only the ones its model declares, `capabilityOverrideProblem`, 400; a model change drops
them). **Effective** = the model's (+ switches) that the interface IMPLEMENTS (`implementedBy`: a slot
declared, or a command `commandRegistry` resolves) — `deviceModelsModel.capabilitiesOf(device)`, null for
an unknown category (then nothing is gated). Gated with `allowsCommand` / `allowsSlot`:
`/api/commands/types` and `sendCommand` (so `shouldDownloadFiles` too), ROS subscriptions (`subscribeDevice`
now takes `deviceModel` + `attributes`), the watchdog's expected slots, MAVLink stream requests and
`_ingest`, MQTT and FlatBuffer ingestion. `getAll` adds `capabilities` (sorted list). Example: griffin on
`ardupilot_mavlink` gets only SaveHome (no `flight_control`), griffin on its mavros interface gets `humidity`,
an x500 there would not. `editDevice` rebuilds the ROS subscriptions on ANY change they depend on (name,
category, model, attributes, camera), deciding on the STORED protocol (a partial PUT carries none).
MAVLink re-reads the device only when its lookup cache expires.

**Mission content follows the vehicle** (plan › F5 · Contenido de la misión). A mission may carry only what its
interface encodes ∩ what its vehicle can do. A capability may own mission items in `capabilities.yaml`
(`mission: { actions, params, waypoint_params }`: `gimbal` → action `gimbal`, `mode_gimbal`, wp `gimbal`; `camera`,
which has no command nor slot → `photo`, `video_*`, `focus_camara`, `zoom_camera`), and `implementedBy` also
counts an interface whose mission profile has one of them. What depends on the vehicle TYPE is `vehicleTypes` on
the item or option in `mission_schema.yaml` (`vtol_transition: [vtol]`). `categoryModel`'s mission getters take an
optional `restrict` (`deviceModelsModel.missionRestrictionOf(device)`: `{ vehicleType, allows(kind, key) }`;
`category.js` cannot import deviceModels, which imports it): without it what the interface encodes, with it what
that device can do. The editor's endpoints (`/api/category/actions|attributeslist|atributesparam|waypointparams|
attributesdefaults/:type`) take `?deviceId=` for it (then the device's own interface; unknown id → 404).
`models/mission/missionValidator.js` is the ONE validator: an item the device cannot carry, a select option it is
not offered, a number out of range, an unknown key → ERROR, located (`{ level, task, device, wp, field, message }`,
`formatProblem` → `T1 · wp 2 · action.gimbal: …`). It refuses every load: manual (`/missions/load` → 400 with
`problems`), automatic (`initMission` → mission error) and the bare `LoadMission` command (MCP; in
`sendToDevice`). A device that does not exist is a WARNING (the task fails and skips its dependents, as before).
`POST /missions/check` returns `{ ok, problems }` without loading: the editor's Validate button.

The category editor (Settings › Category) edits these four blocks (`subscribers`, `services`,
`actions`, `publishers`). The keys and types it offers come from `GET /api/category/messages`
(`categoryModel.getMessagesType`), DERIVED from what `msgDecode` can decode plus what categories
already declare — there is no hand-kept list (the old `messages.yaml` drifted and was removed).
`PUT`/`POST /api/category/:category` validate the WHOLE catalog with the change applied BEFORE
writing (an edit must not leave an invalid file nor break another interface): `400 { error,
errors: [{path, message}] }`, `404` unknown, `409` already exists. Saving rewrites only that
category's file, so its comments (and only its) are lost.

**Message Decoding** (`models/msgDecode.js`, NOT under `ros/`, the mirror of `msgEncode`): a
registry, not an if-chain. `decodeMsg` serves every transport: rosbridge (`ros.js`) and MQTT
(`MqttServer.js`) with ROS-shaped JSON, robofleet with the ROS message unpacked from its Flatbuffer
(`decodeFbViaCategory`), and MAVLink (`MavlinkServer.js`) with `mavlink/<MSG>` types (see MAVLink Protocol).
A decoder is picked by `(slot, messageType)` — the `subscribers` key plus the `messageType` that
entry declares — and returns a `TelemetryPatch` (typedef in `models/positions/positionReducers.js`),
which `positionsModel.updatePosition` merges through ordered reducers.

- `sensor_msgs/NavSatFix` → GPS position (slots `position`, `position_rtk`)
- `px4_msgs/VehicleStatus` → `armState` + `flightMode` (canonical, see below), failsafe
- `sensor_msgs/Imu` → Heading from quaternion
- `sensor_msgs/BatteryState` (and its `*/BatteryState*` look-alikes) → Battery percentage

**Several slots with lat/lon: source arbitration** (`gatePosition`, `models/positions/positionReducers.js`;
test `test/test-position-sources.js`). DJI categories subscribe `position` (GNSS) AND `position_rtk`
(`/dji_osdk_ros/rtk_position`); there is no `*_rtk` category. It is a param of each subscriber entry, never a
branch on a slot or category name: `priority` (higher wins, default 0) and `fresh_ms` (how long a patch of that
slot keeps blocking lower priorities, default `POSITION_FRESH_MS` = 3000). `decodeMsg` stamps
`positionSource`/`positionPriority`/`positionFreshMs` on a lat/lon patch ONLY when the entry declares one of
them, so a category without them behaves as before. A lower-priority patch is dropped while a higher one is
fresh (server clock, injected as `now`); when the RTK goes quiet GNSS takes over, and when it returns it wins
at once. A dropped patch loses `latitude/longitude/altitude/deviceTime` (altitude is taken as each source
gives it, the datum is open in `TODO.md`), so `POSITION_RECEIVED` and mission tracking only see the position
that was applied. A patch with lat = lon = 0 is never applied and never counts as fresh (any category). The
winner is `position.attributes.positionSource`; a change raises NO Event (a DB row plus a snackbar per switch; near towers the RTK fix flaps). Each source also keeps its own
last fix in `attributes[entry.attribute ?? slot]` (DJI: `position_gnss`, `position_rtk`) as `{ latitude, longitude,
altitude, at, fix }`: `at` = server ms of its last message (age = now - at), `fix: 'nofix'` when it sent 0,0 (then
no coordinates). Winner or not, so the client can show both. The slot that may be absent
(`position_rtk`) needs `watchdog: false`.

**Vehicle state is canonical, never a firmware label.** Three independent axes in
`position.attributes`, vocabulary in `config/status.js`: `armState` (`ARM_STATE`), `flightPhase`
(`FLIGHT_PHASE`, where it is) and `flightMode` (`FLIGHT_MODE`, how it is flown; `isAutonomous(mode)`
/ `controlLevelOf(mode)` say who is in control, derived from the mode). A VTOL has a fourth,
`vtolState` (`VTOL_STATE`: MC / FW / transitioning; MAVLink `EXTENDED_SYS_STATE.vtol_state`), written
ONLY when the vehicle reports one: a multirotor has no such key, not an UNKNOWN. A decoder translates its
message's own codes through `models/vehicleStates.js` (NOT under `ros/`: rosbridge, MQTT and robofleet
all call it) and keeps the raw code in `flightModeRaw` / `flightPhaseRaw`. `UNKNOWN` = the source says
nothing, `OTHER` = a mode the GCS does not tell apart. Consumers compare against the constants, never
against a string: `missionSignals` does, and the client mirrors them (with the text to show) in
`client/src/common/util/vehicleState.js`, guarded by `test/test-vehicle-state-mirror.js`. A new
firmware = a new adapter in `vehicleStates.js` plus a test; no consumer changes. A reducer writes only
the keys a message carries (DJI `flight_status` brings arm + phase, DJI `display_mode` the mode, PX4
`vehicle_status` arm + mode). Two slots can share a messageType (`flight_status` and `display_mode`
are both `std_msgs/UInt8`): the slot, not the type, says which axis it is, so give each its own.

**What differs per category is a param, never a branch on the category name.** `ros.js` hands the
decoder the category's own subscriber entry as `params`; the decoder reads, with defaults:
`battery.scale` (× to percent, default 1 — `100` when the message is 0-1), `gimbal.unit`
(`rad` → degrees), `imu.heading_offset` (default 90). A 0-1 battery WITHOUT `scale: 100` is
rounded to 0 or 1 by the position cache and also reaches the external planner as
`battery_level` (`mission.js`) ~100× too low. At boot, `categoryModel` checks every
subscriber's `(slot, messageType)` against the registry (`hasDecoder`); `LOG_LEVEL=debug` makes
a gap throw. A slot declared in the YAML that nothing consumes gets an explicit `ignore(...)`.

**Per-slot telemetry watchdog** (`models/inspect/telemetryWatchdog.js`, test `test/test-telemetry-watchdog.js`).
`DeviceHealthMonitor` only sees the device as a whole, so one live slot keeps it ONLINE while
`battery` never arrives. The three ingestors call `telemetryWatchdog.record(deviceId, slot)` next to
`rawTap.publish` (before the decode); a timer (`TELEMETRY_CHECK_INTERVAL_MS`) compares every subscribed
slot of the category against `TELEMETRY_SLOT_TIMEOUT_MS`. **On by default for every slot**: an
event-driven one (`vehicle_command_ack`, `threat`) needs `watchdog: false` on its subscriber entry, a
slow one `timeout_s`, or it is a permanent false alarm. A slot that declares `rate_hz` (MAVLink) and no `timeout_s` waits `max(TELEMETRY_SLOT_TIMEOUT_MS, 3 / rate_hz)` (`slotTimeoutMs`), so changing the rate moves the timeout with it. `stale` = arrived then stopped (sensor/node
down), `missing` = never arrived since the device showed up (config: topic name, PX4 version suffix).
The expected slots follow the SAME camera rule as the ROS subscription (`subscribesCamera` in
`category.js`). A device silent as a whole is skipped (that is OFFLINE, not N slot warnings). Only
transitions are reported: a `warning`/`info` Event (snackbar) plus an immediate `DEVICE_UPDATED`. The
state reaches the client as `device.telemetry` (`{ slot: 'stale' | 'missing' }`, `{}` when fine),
added by `DevicesModel.getAll`, shown as a chip in `StatusCard`. It also counts messages for a
smoothed per-slot rate: `GET /api/devices/:id/telemetry` → `{ slot: { health, hz, ageMs } }`, polled
by the device messages page. Timestamps and counts only, never content — the content is `rawTap`'s. Dependencies are injected in
`start()` from `server.js`: `models/devices.js` imports the watchdog, so it cannot import back.

**Message Encoding** (`models/msgEncode.js`, NOT under `ros/`): a registry by msgType, the mirror of
`msgDecode` on the way out. The structure of a message depends only on its msgType and is the same for
every protocol (rosbridge JSON, the same JSON over MQTT, robofleet = ROS messages packed in Flatbuffers),
so it is encoded ONCE, before the gateway: `commandsModel.sendToDevice` (commands) and the raw routes
`/ros/publish` and `/ros/action_send_goal`. The transport primitives (`callService`, `sendActionGoal`,
`PubRosMsgOnce`, `MqttServer`) receive the message already encoded and never encode. Three layers for a
command: `commandRegistry` (what is sent, domain) → `encodeMsg(msgType)` (its shape) → gateway (packing).
An encoder that needs several messages in order on the same endpoint returns `sequence([...])`
(`commandEncoders.js`; PX4 Gimbal = take control with CONFIGURE, then PITCHYAW): `sendToDevice` sends
them in order and stops at the first one that fails. NaN goes to rosbridge as the bare `NaN` literal
(`stringifyKeepingNaN`, the `transportFactory` of `rosConnection.js`); MQTT's JSON still turns it into null.
Every msgType a command endpoint uses must be registered (an encoder or an explicit `passthrough`);
`commands.js` checks it at boot. `test/api/json/msg_encode_golden.json` froze the old `rosEncode` output.

### ROS Module Layout (`models/ros/`)

The ROS integration is split by responsibility. **`ros.js` is the facade and the
ONLY file allowed to bridge the ROS world and the business world** — it imports
the business controllers (`devices`, `mission`, `positions`) and wires their
effects into the pure transport primitives. The transport files below must stay
free of business-controller imports.

| File                  | Responsibility                                                                                                                                                                                                                                |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ros.js`              | Facade (`rosModel`). Resolves `uav_id → {name, category}`, resolves per-category message config via `categoryModel` (`resolveCategoryConfig`), builds every ROS name (`buildDeviceName`), injects business callbacks, delegates to primitives |
| `rosConnection.js`    | Connection lifecycle: connect, auto-reconnect (30s), inject connected/disconnect handlers                                                                                                                                                     |
| `rosSubscriptions.js` | Subscription primitives: `subscribeTopics`/`unsubscribeKey`/`unsubscribeAll`/`subscribeOnce`. Keeps `activeSubscriptions` (opaque subscriptionKey → listeners); no category/camera/topic-name knowledge |
| `rosPublishers.js`    | Publish primitives: `PubRosMsgOnce` (validated one-shot) and `startPeriodic`/`stopPeriodic`/`stopAllPeriodic` (advertised topic + interval, e.g. the GCS heartbeat). Keeps `activePublishers` (opaque key → topic + timer); no category/topic-name knowledge |
| `rosServices.js`      | ROS service calls: `callRosService` (primitive) + `callService` (receives the already-built service name + type; no config lookup)                                                                                                            |
| `rosAction.js`        | ROS2 action lifecycle + registry (see below). Device-layer methods receive the already-built `actionServerName`; no config lookup nor name-building                                                                                           |
| `rosInspect.js`       | Read-only rosapi introspection (topics/services/types/action servers)                                                                                                                                                                         |
| `rosValidateMSG.js`   | Message structure validation against type maps                                                                                                                                                                                                |
| `ros2ActionClient.js` | Alternative action client (`callOnConnection`); currently unused                                                                                                                                                                              |
| `index.js`            | Module barrel exports                                                                                                                                                                                                                         |

**Config resolution lives ONLY in the facade.** `categoryModel` (in
`models/category.js`) is the single source of truth for the interface files
(`config/devices/interfaces/`) — it is the ONLY module that reads them (through
`models/interfaces/`). No file under `models/ros/` reads them.
The facade resolves per-category message config through one helper and builds
every ROS name through another:

- `resolveCategoryConfig(category, block, type)` → `categoryModel.getCategory(category)?.[block]` (with `type`, the single entry). `block` is `'subscribers' | 'publishers' | 'services' | 'actions'`. Using `categoryModel` (not a frozen `readDataFile` snapshot) means a runtime category edit is picked up immediately.
- `buildDeviceName(name, entry)` → `/${name}${entry.name}`. Device names are stored WITHOUT a leading slash and every config `name` starts with one, so every resulting topic/service/action name is absolute (`/agv_1/odom`). All four blocks use this one helper, so a device's topics, services and actions resolve identically. Both live in `models/ros/rosNames.js` (pure). The SUBSCRIBERS use `buildTelemetryName`, which is the same unless the device has `attributes.slow` (ROS-only attribute group 'ros'): its companion runs a relay that republishes the telemetry at a lower rate, read from `/<device>/slow<topic>` (only those cross the link; rosbridge's own `throttle_rate` would drop them on the GCS side, after they crossed). Services, actions and publishers never move. It replaced the `px4_mavros_slow` interface (`common/migrations/deviceSlowRelay.js`).

**Periodic publishers.** A category may declare a `heartbeat:` block in `devices_msg.yaml` (`name`, `messageType`, `rate_hz`, `message`) — NOT under `publishers:`, which feeds `commandRegistry` and would expose it as a command. `subscribeDevice` starts it through `startPeriodic` under the device id and `unsubscribeDevice` stops it; `message` holds the constant fields and the facade stamps `timestamp` each tick. px4_ros2 uses it for the `telemetry_status` GCS heartbeat PX4 needs to arm when `NAV_DLL_ACT > 0`.

**Two-layer pattern (topics, services and actions).** Each has a pure primitive
that takes already-resolved ROS names/types. The facade resolves the config +
builds the name, so the primitives never touch `devices_msg`, `categoryModel`,
or name-building:

- Topics: `subscribeTopics({key, topics})` ← `subscribeDevice({id, name, category, camera})`. The facade resolves the subscriber list, decides camera gating, builds each topic name with `buildDeviceName`, and closes `deviceId`/`category`/`slot` into each topic's `onMessage`; the primitive only opens/closes ROSLIB listeners under an opaque key. `subscribeTopic({topic, messageType, onMessage})` reuses the same primitive for ad-hoc, device-less subscriptions keyed by topic name.
- Services: `callRosService({service, messageType, message})` ← `callService({name, type, service, serviceType, request})` — the facade passes the built `service` name + `serviceType`; the primitive only encodes + calls.
- Actions: `sendRosActionGoal({actionServerName, actionType, message})` ← `sendActionGoal({actionServerName, actionType, type, message})` — the facade builds `actionServerName`; the device-layer method only encodes + delegates.

The facade exposes both layers; the device-layer resolves `uav_id` first. The
device-layer facade methods carry a `Device` suffix to set them apart from the
primitives and the `*Ros*` raw methods: `callServiceDevice`,
`publishTopicDevice`, `sendActionGoalDevice`, `getActionStatusDevice`,
`cancelActionDevice` (each takes `{uav_id, type, ...}`). `rosController` mirrors
those names for its non-HTTP callers (e.g. the ROS gateway); the
HTTP handlers keep the `*Handler` suffix. `unsubscribeDevice(id)` unsubscribes
one key, or ALL keys when `id < 0` (`-1` on disconnect).

**ROS2 Actions** (`rosAction.js`): unlike fire-and-forget topics/services,
actions are long-running (goal → feedback → result/cancel), so their state lives
in a `registry` Map keyed by action-server name (one entry per server). **Invariant:
every cancellation goes through the registry via `cancelAction`/`cancelRosAction`** —
never cancel by raw goalId outside it, or the registry goes stale. HTTP routes:
`/ros/action_*` (device-layer, by `uav_id`+`type`) and `/ros/action_*_raw`
(primitive, by raw action name — used by the MCP server).

### LLM Chat Integration

**Architecture** (`models/chat/`):

- **Message Orchestrator** (`chat.js`, `MessageOrchestrator`): central turn loop — builds context, calls the LLM, dispatches tool calls (MCP + local), recurses until the turn finishes or hits the iteration cap
- **LLM Factory** (`handlers/llmFactory.js`, `LLMFactory`): picks a handler by `LLMProvider` (`config.js`). Three transports, not one class per vendor:
  - **Responses API** → `openaiHandler.js` (OpenAI only — no third party implements it, so it keeps server-side conversations, `reasoning` items and `allowed_tools`)
  - **Chat Completions** → `openaiCompatibleHandler.js` (`ollama`, `llamacpp`, `openai-compatible`), parametrized by `baseURL` + a preset from `handlers/compatibleProviders.js`
  - **Vendor APIs** → `geminiHandler.js`, `antropicHandler.js`
- **Adding a local/compatible server** (vLLM, LM Studio, OpenRouter…) is an entry in `compatibleProviders.js`, NOT a new class.
- **Capability rule** (`compatibleProviders.js`): `capabilities` may only shape the OUTGOING request. Parsing is unconditional and defensive, because a server can stop honouring a declared capability without notice (llama.cpp#20198 turned `arguments` into a parsed object). An `if (capabilities.x) parseA() else parseB()` is a subclass hiding in an object.
- **Two parsing invariants** for compatible servers: detect tool calls by the PRESENCE of `tool_calls` (llama.cpp reports `finish_reason: "tool"`, not `"tool_calls"`), and accept `arguments` as string OR object.
- **Agent profiles** (`agents/*.md`): system prompts as Markdown files with YAML frontmatter, loaded by `agents/index.js`. Each `.md` is a distinct agent persona/tool-set (e.g. `default.md`/`default2.md` general chat, `defaultFast.md`/`plannerFast.md` lighter/faster profiles, `planner.md` the full mission-planning agent — see `mip_planner`/MCP `submit_mission_plan` flow, `verification-mission.md`, `agv.md` for ground vehicles). **`planner.md` is the source of truth; `plannerFast.md` must stay in sync with it** — see PR history for past drift bugs.
- **Sub-agents** (`subAgentManager.js`, `subAgentRegistry.js`): a chat can spawn child agents (e.g. a verification pass) tracked in an in-memory registry keyed by parent `chatId`; `getContextParams`/`removeSubAgent` let the orchestrator resolve/clean them up
- **Chat history** (`chatHistoryManager.js`, `messageProjection.js`, `turnContext.js`): persistence and per-provider message-shape projection, so history stored once in DB can be replayed into any provider's expected format
- **MCP Client** (`mcpClient.js`): Model Context Protocol client for UAV-specific tools, backed by the `mcp_server/` submodule
- **Usage tracking** (`chatEvents.js` + `ChatUsage` model): token usage tracked per request/turn/chat

**MCP Server Integration:**
The project includes an external MCP server as a git submodule (`mcp_server/`):

- **Location**: `mcp_server/` directory (git submodule from https://github.com/arpoma16/muav_gui_mcp.git)
- **Purpose**: Provides UAV-specific tools for LLM integration (mission planning, device control, telemetry queries)
- **Protocol**: Supports both STDIO and SSE transport modes
- **Features**:
  - Auto-reconnection with exponential backoff (1s → 16s over 5 attempts)
  - Detects connection errors (ECONNREFUSED, ECONNRESET, socket hang up)
  - Integrated with server via `mcpClient.js`
- **Development**: Can be debugged independently using VS Code Debug panel or MCP Inspector
- **Configuration**: Enable/disable via `MCP_ENABLE` in `.env`, configure transport in `MCP_CONFIG`

**Tool Calling Loop:**

- Max 6 iterations to prevent infinite recursion
- Tools: Mission planning, device control, sensor queries
- Tool results fed back to LLM for context-aware responses

**Human-in-the-loop gate for flight tools** (`toolApproval.js`, `toolApprovalStore.js`,
`approvalSweeper.js`, `ChatToolApproval` model):

Tools in `TOOL_APPROVAL_REQUIRED` park the whole tool-call batch until an operator
answers. The invariants, in order of how badly they break things if ignored:

1. **The approval binds to the frozen, serialised input — never to the intent.** The
   resolved input (context params already merged) is hashed at park time, and re-checked
   against the value about to run. Resuming NEVER re-asks the model: a model asked twice
   can answer twice, and the operator approved the first answer.
2. **Free text approves nothing.** Only a structured `{requestId, optionId}` on
   `chat:tool_approval_response` decides. With several requests open, no amount of
   "yes, go ahead" disambiguates which one it meant.
3. **A stale answer never re-opens a decision.** `resolve` is an atomic
   `UPDATE ... WHERE status='pending'`, so with several operators the first write wins
   and the rest are told. `claimExecution` is the same trick on `executedAt IS NULL`,
   which is what stops a double click flying the mission twice.
4. **Expiry is swept, not just recorded.** A parked turn holds a `function_call` with no
   result, which providers reject on the NEXT request — so `approvalSweeper` expires and
   then RESUMES the turn with rejected results, keeping history well-formed.
5. `rejected` ≠ `failed`: a denied call never ran. Collapsing them destroys the audit trail.

The parked turn releases the chat lock, so the operator can still talk (and answer).
Pending requests live in the DB, so they survive a reload and a server restart.

**Inbound/EventBus Integration:**

- `CHAT_USER_MESSAGE` (WS inbound) → `WebsocketInboundRouter` → `chatController.processMessage` (direct dispatch)
- `CHAT_CREATED` → notifies the client a new chat was created (outbound via subscriber)
- `CHAT_ASSISTANT_MESSAGE` → Broadcasts response to clients via WebSocket

**eve Agent Bridge (`eveClient.js`, `eveEvents.js`, `eveRelay.js`, `eveOrchestrator.js`):**

A second, parallel orchestrator that talks to an `eve` agent (`gcs_eevee_assistant`, a sibling
checkout at `../gcs_eevee_assistant` relative to `llm_planner_gcs/`) over its `<base>/v1/session` HTTP
API. Opt-in per chat via `chat.metadata.engine === 'eve'` (set at creation, `POST /api/chat/chats
{engine:"eve", agent?}`); everything else keeps using `MessageOrchestrator`/`mcpClient.js` unchanged and
`chatController._resolveEngine` is the only branch point. eve is its own full orchestrator (tool loop,
sub-agents, sandbox): this bridge does not reimplement any of that. Plan, decisions and what is still
unverified: `eve-implementation.md` (repo root of `multiuav_gcs/`).

**It does not wait for the answer.** A turn can wait on a human for hours, so `EveOrchestrator.processMessage`
only sends the message and makes sure `EveRelay` is reading; what comes back reaches the client through the
same events the legacy path uses (`CHAT_ASSISTANT_MESSAGE`, `CHAT_BUSY`, `CHAT_TOOL_APPROVAL_*`) plus
`CHAT_ACTIVITY` (running / planning / awaiting_input, only eve reports it).

- `eveEvents.js`: a PURE state machine over the event stream (`reduceEvent(state, event) → {state, effects}`).
  A turn ends on `turn.completed|failed|cancelled`. `turn.waiting {on:"input"}` parks it on a person (the
  reader stops); `{on:"tasks"}` is sub-work and the turn is still running. The answer is the LAST
  `message.completed` of the turn (a turn appends one per step; `finishReason` cannot tell them apart).
  Text deltas (`message.appended`, `action.input.appended`: ~240 per turn) are ignored.
- `eveRelay.js`: one background reader per chat. Persists the cursor and the turn state in
  `chat.metadata` (`eveStreamIndex`, `eveTurn`) after every event that changes something, reconnects with
  backoff, dedupes by `meta.id`. At-least-once: a crash between an effect and the cursor save replays that
  one event (the audit is idempotent, a persisted item is not).
- `eveClient.js`: HTTP. Agents come from `config.EveAgents` (eve cannot list them). **Never build a URL from
  an event's `streamPath`**: eve fills it with its agent-local route (`/eve/v1/...`), a 404 under `dev:all`.
- Persisted items reuse the legacy shapes (`function_call`, `function_call_output`, `text`) so REST history and
  `ChatMessages` need no change. Interim `message.completed` texts and task events are NOT persisted.

**eve's stream is durable and replays from event 0 on every `GET .../stream` unless a `startIndex` is
passed.** The relay always passes its cursor. Without it every follow-up re-reads the previous turn.

**Approvals.** `input.requested` → `CHAT_TOOL_APPROVAL_REQUESTED` with the same `InputRequest` the legacy
engine sends, plus `options` (eve: `approve`/`cancel`) and no `expiresAt`/`inputHash`. The answer goes
`chat:tool_approval_response` → `chatController.respondToApproval` → (eve) `EveOrchestrator.respondToInput` →
`POST inputResponses`. Rules, in order of how badly they break things:

1. **Fail-closed on `decided`.** The operator's decision is written to `ChatInputAudit` BEFORE it is sent; if
   that write fails, nothing is sent, the card stays (`CHAT_TOOL_APPROVAL_FAILED`) and they can retry.
   Everything else in the audit is best-effort (`requested`, `executed`, and all of the legacy engine, where
   `ChatToolApproval` already is the gate).
2. **Free text approves nothing.** Sent while an approval is open it CANCELS it (eve outcome `ignored`).
3. **An eve card leaves when eve confirms (`input.resolved`)**, not when the operator clicks.
4. The open approvals live in `metadata.eveTurn.pending`, so they survive a reload and a restart.

`ChatInputAudit` (`models/chat/inputAudit.js`) is an append-only record for BOTH engines, one row per event
(`requested|decided|executed`), no FK to `Chat`. Unique on `(engine, requestId, kind, source)`: replaying a
stream cannot duplicate a row. `responderPrincipalId` stays null (the GCS has no users).

Two lists decide which tools need a person: `TOOL_APPROVAL_REQUIRED` here and `FLIGHT_CRITICAL_TOOLS` in the
eve repo (`connections/multiuav-gcs.ts`). `test/test-eve-flight-tools-contract.js` fails if this server gates a
tool eve does not.

**Retention (read before production).** For `private`/`unknown` audiences outside development eve asks its World
to delete a session when it ends (retention 0): finished turns become unreadable (`410 session_expired`, the
relay reports it and stops). Our persisted items are then the only copy. `experimental.workflow.retention:
"default"` in the agent's `agent.ts` keeps eve's copy; its duration is not documented.

Requires, for local dev: `mcp_server` in `http` transport (`:3001/mcp`) and `gcs_eevee_assistant` running
(`npm run dev:assistant` on `:2000`, or `dev:all` on `:3300` with one path per agent; needs Node 24, this
server runs on Node 20). Env: `EVE_ENABLE`; `EVE_AGENTS` (JSON `{"assistant":"http://localhost:3300/eve/assistant"}`)
or, for the single `dev:assistant` agent, `EVE_URL` (default `http://127.0.0.1:2000`; the `/eve` mount is added
if missing); `EVE_DEFAULT_AGENT`. In production eve rejects everything but Vercel OIDC unless the agent's
`channels/eve.ts` configures `eveChannel({auth})` with service auth.

Not implemented yet: text streaming (deltas), the child session of a subagent (`agent.started.sessionId`), the
cancel button (`EveOrchestrator.cancel` exists, no route), activity after a page reload, free-text questions.
`subagent.called`/`childSessionId` do NOT exist any more in eve (removed): a subagent shows as `task.started`
(+ `agent.started` when it opens a session).

**MCP Reconnection System:**
The MCP client implements automatic reconnection:

- Detects disconnections from server restart or network issues
- Exponential backoff: 1s, 2s, 4s, 8s, 16s (configurable)
- Max 5 reconnection attempts (configurable)
- Retries failed tool executions after successful reconnection
- Manual reconnect/reset via `mcpClient.reconnect()` and `mcpClient.resetReconnectAttempts()`

## Important Patterns and Conventions

### EventBus Safety

Always use safe methods to prevent crashes:

```javascript
// Good
eventBus.emitSafe('POSITION_UPDATED', data);
eventBus.onSafe('POSITION_UPDATED', handler);

// Avoid (no error handling)
eventBus.emit('POSITION_UPDATED', data);
```

### State Machine Updates

When modifying mission flow, check all of:

1. State machine definition (`models/mission/missionSM.js` / `missionExecutionSM.js`)
2. Command handlers in `models/commands/commands.js`
3. The scheduler's readiness rules (`models/mission/taskScheduler.js`) and the two flows in `models/mission/mission.js` (`initMission`, `loadMissionManual`/`commandMissionManual`)
4. Every task must end in a terminal status on every path (success, failure, cancel, restart): a task left in `init` of a dead mission keeps its device "busy" for new mission requests forever

### ROS Message Handling

When adding new device types:

1. Add a file `config/devices/interfaces/<category>.yaml` (`subscribers`, `services`, and/or `actions`); put any per-category scale/unit/offset on the subscriber entry itself (a NEW param also goes in `schemas/zod/interface.js`)
2. Register a decoder for the `(slot, messageType)` pair in `models/msgDecode.js` (or `ignore(...)` it) — boot validation and `test/test-devices-msg.js` fail on a pair with none
3. If it takes commands: register the command encoder in `models/commands/commandRegistry.js` (by msgType, or by key for a generic msgType) and the message encoder for that msgType in `models/msgEncode.js` (or `passthrough` it) — the boot check and `test/test-command-registry.js` fail otherwise
4. Subscriptions are created automatically — the facade (`ros.js`) resolves the config via `categoryModel` and hands built topics to `models/ros/rosSubscriptions.js`; no per-device code needed

### WebSocket Message Format

```javascript
// Server → Client
wsController.sendMessage({ type: 'positions', positions: {...} });

// Client processing (SocketController.jsx)
dispatch(sessionActions.updatePositions(data.positions));
```

## Device Communication Protocols

**Command routing** (`models/gateways/`): `commandsModel.sendToDevice` picks a gateway by
`device.protocol` (a missing protocol means `ros`) and calls `gateway.sendCommand({ device, endpoint,
message })` — `endpoint` (`{ block, key, msgType }`) is already resolved by `commandRegistry`, so the
ROS gateway reads the transport off `endpoint.block` instead of guessing. One gateway per protocol (`rosGateway`, `flatbufferGateway`, `mqttGateway`, `mavlinkGateway`),
registered in `gateways/index.js`; the caller never branches on the protocol. A new protocol is a
new gateway plus an entry in `DEVICE_PROTOCOL` (`schemas/zod/devices.js`) — `test/test-gateways.js`
fails if the two drift apart. Gateways resolve their controller/server when a command is SENT, not
at registration: `controllers/ros.js` reaches `commands.js` through its imports, so a default
evaluated at registration hits a circular-import TDZ.

**Protocol vs message format.** The PROTOCOLS are four (`ros`, `mqtt`, `robofleet`, `mavlink`): each
needs its own software on the robot and its own gateway here, and `device.protocol` picks it. A
CATEGORY only says which MESSAGE FORMAT it speaks, derived from its `messageType`s
(`categoryModel.messageFormatOf`): `mavlink` (`mavlink/<MSG>`, carried only by mavlink) or `ros_msg`
(carried by ros, mqtt and robofleet alike). `GET /api/category?protocol=` lists the categories the
protocol can carry (no param: all, as before); `POST`/`PUT /api/devices` refuse a category the protocol
cannot carry, a mavlink device without `attributes.systemId`, or one whose `systemId` another mavlink device already has (`categoryModel.deviceProblem` + `systemIdTaken`, in the controller
because zod schemas do not import models; a `PUT` is checked on the device it leaves, stored + partial).
`GET /api/server/protocol` returns the four as `{ name, enabled }`: the device form greys out the
ones this server does not run. The server state on the WS carries `mqttState` only when MQTT is
enabled (`controllers/websocket.js` `serverState()`).

**Per-device attributes** (`Device.attributes`, JSON): settings specific to each robot that depend on
something else about it (mavlink: `{ systemId }`). They are declared as GROUPS in `ATTRIBUTE_GROUPS`
(`schemas/zod/devices.js`): `appliesTo({protocol, category, attributes})` says when a group applies and `shape`
(zod fields) which keys it adds, so a parameter can depend on the protocol, the category or another attribute's
value. A device's attributes are the strict union of the groups that apply to it (an unknown key is a 400, path
`['attributes', ...]`; two groups declaring one key for the same device throw). A new per-robot parameter is a
new group there, no new column. `validateDevice` coerces them on create; on `PUT` the controller validates the
attributes of the device it leaves: stored keys no group applies to any more are DROPPED (`pruneAttributes`,
e.g. mavlink's after a protocol change) and the ones sent are merged on top. They are public (`publicFields`):
never put a secret in them. The client form still hardcodes the mavlink field (`DeviceIdentityEditor`).
**A device has no `ip`, `user` or `pwd`**: the address of a camera's stream is `camera[].host` (required unless
the camera is `Websocket`; `camera.js` builds the RTSP URL from it, the client's `CameraDevice` uses it for
`WebRTC_env`), and the file-download credentials are in `files[].url`. Migration:
`common/migrations/deviceCameraHost.js`.
`getBySystemId` filters in JS because JSON paths in a WHERE differ per dialect. Migration from the old
`systemId` column: `common/migrations/deviceAttributes.js` (native `DROP COLUMN`, idempotent).

**ROS Protocol** (`protocol: 'ros'`):

- Requires ROS bridge running on `ws://127.0.0.1:9090`
- Launch: `roslaunch aerialcore_gui connect_uas.launch`
- Topics auto-subscribed based on device category

**FlatBuffer Protocol** (`protocol: 'robofleet'`):

- Direct WebSocket connection to fleet
- Binary encoding for bandwidth efficiency
- Used for non-ROS devices or optimized deployments
- robofleet carries ROS messages packed in Flatbuffers generated from the same `.msg`/`.srv`
  (`fbmsglib`, e.g. `fb.aerialcore_common.ConfigMissionRequest`). So `fbEncode.js` receives the ROS
  message already encoded by `msgEncode` and only PACKS it (rosbridge's `{ data: [...] }` arrays become
  plain vectors). Only `configure_mission` with the ROS1 ConfigMission structure and `command_mission`
  (`std_srvs/SetBool`) have a schema; another mission type (ROS2 LoadMission, PSDK) is rejected with an
  error instead of packed as ConfigMission. The robofleet topics keep their own names
  (`/<uav>/configureMission`, `/<uav>/commandMission`), independent of the devices_msg keys. Not
  verified against a real robofleet device.

**MQTT Protocol** (`protocol: 'mqtt'`, opt-in with `MQTT_CONNECTION=true`, broker at `MQTT_URL`, default `mqtt://127.0.0.1:1883`):

- `models/mqtt/MqttServer.js` (started from `server.js` via `initMqttServer`), mirror of `FlatbufferServer`. Plan and open risks: `MQTT_PLAN.md`.
- Topic contract lives ONLY in `models/mqtt/mqttTopics.js`: `muav/<device>/pub/<slot>` (telemetry), `muav/<device>/srv/<name>/{request,response}` (service calls), `muav/<device>/status` (retained presence, plain `online`/`offline`). `<slot>` / `<name>` are the keys of `subscribers` / `services` in `devices_msg.yaml`.
- Telemetry reuses `decodeMsg` unchanged: the drone publishes the same JSON shape rosbridge does. A payload with bare `NaN`/`Infinity` is parsed with those as `null` instead of being dropped.
- Service calls are enveloped (`{id, ts, request}` → `{id, success, response, error}`) and correlated by `id`. Unlike robofleet, `sendCommand` resolves only when the drone answers or the GCS timeout fires. `envelope.success=false` means the CALL failed; the service's own verdict (`response.success || response.result`) stays inside `response`.
- Only `services` are bridged. Publishers and actions on an `mqtt` device answer a warning (out of scope for v1).
- Tests: `test/test-mqtt-server.js` runs against a fake client (no broker).

**MAVLink Protocol** (`protocol: 'mavlink'`, opt-in with `MAVLINK_CONNECTION=true`, links in `MAVLINK_LINKS`, default `udp:0.0.0.0:14550`). PX4 and ArduCopter on `node-mavlink` (pinned: its traps below were measured on 2.3.0). Plan, SITL results and open items: `MAVLINK_PLAN.md` — read its "Resultados F0" and "Estado Fn" before changing anything here.

- `models/mavlink/`: `links.js` (UDP listens, one parser per remote `address:port`; TCP connects; serial not yet), `MavlinkServer.js`, `mavlinkMessages.js` (registry, GCS identity 255/190, builders). Started from `server.js` via `initMavlinkServer`.
- **A vehicle is its `device.attributes.systemId`** (1..254), on ANY link: the route back is learned from the autopilot HEARTBEAT (`autopilot != INVALID`, `type != GCS`; components and other GCS are ignored). Presence from that heartbeat (offline after 5 s); `device.status` still comes from positions (`DeviceHealthMonitor`). The GCS heartbeats at 1 Hz; PX4 answers TEMPORARILY_REJECTED to commands until it heard it for ~1 s, so commands wait for that.
- **node-mavlink traps** (they fail silently, the ACK still says ACCEPTED): the header is `sysid`/`compid` (no `srcSystem`); in `CommandLong`/`CommandInt` the params AND x/y/z are `_param1.._param7` (`msg.param1 = …` sends 0; PX4 once got lat 0, lon 0) — only `commandLong`/`commandInt`/`missionItemInt` in `mavlinkMessages.js` build them, and every new command gets a serialize→parse round-trip test; `AUTOPILOT_VERSION` is in the `standard` dialect.
- **Telemetry**: categories `px4_mavlink` / `ardupilot_mavlink` declare `subscribers` with `messageType: 'mavlink/<MSG>'` (no `name:`); the plain view (`toPlain`: spec field names, wire integers, BigInt made JSON-safe) goes to rawTap, the watchdog and `decodeMsg`. `rate_hz` on a subscriber is a stream asked with SET_MESSAGE_INTERVAL when the vehicle comes online and again while missing: ArduPilot sends NO telemetry until asked, PX4 sends position at ~50 Hz. The firmware is a fact of the HEARTBEAT (`mavlinkFlightMode({autopilot, type, customMode})`, never the category name). `home_position` follows HOME_POSITION: both firmwares move home on every arming.
- **Commands**: the endpoint msgType is the dialect (`mavlink/PX4_COMMAND`, `mavlink/ARDUPILOT_COMMAND`, `models/commands/mavlinkCommand.js`, registered by `(key, msgType)`); the server refuses to send when the heartbeat reports another autopilot. Same number, different meaning: NAV_TAKEOFF altitude is AMSL on PX4 (it ignores the COMMAND_INT frame) and relative on ArduCopter (only GLOBAL_RELATIVE_ALT, else DENIED); PX4 reads the DO_REPOSITION yaw in RADIANS, ArduCopter ignores it (a CONDITION_YAW follows). Anything with lat/lon goes as COMMAND_INT. **Hold/Pause/Stop on ArduCopter is BRAKE, never LOITER**: LOITER follows the throttle stick and with no RC it descends to the ground. ACCEPTED = taken, not finished; a refusal carries the STATUSTEXT reason (chunks glued by `id`).
- **Missions**: `configure_mission` with `mavlink/PX4_MISSION` / `mavlink/ARDUPILOT_MISSION` (family `MAVLINK_SYMBOLS`, profile `mavlink`); `missionEncodeMavlink.js` builds the items (the frame follows the task's `altitude_ref`: relative, the default and what every mission is today (editor and planner) → `MAV_FRAME_GLOBAL_RELATIVE_ALT`, over the home the autopilot holds; `amsl` → `MAV_FRAME_GLOBAL`. A relative height sent as AMSL put the takeoff at 5 m above sea level, which PX4 refused and ArduPilot would have flown below the ground; the client's `RuteConvert` must keep the field (`test-mission-convert-client.js`). ArduPilot home placeholder at seq 0, no NaN where `AP_Mission::sanity_check_params` refuses it) and `wpSeqs`, the seq of each waypoint — the extra items (speed, gimbal, camera) must not count, MISSION_ITEM_REACHED becomes `wp_reached` = index + 1 through it, and it is reset to 0 on every load. The GCS detects the end itself (last waypoint; plus disarmed for RTL/land) and calls `deviceFinishMission`. Two PX4 traps: an upload with no item flagged `current` resumes at the cursor of the PREVIOUS mission (`mission_base.cpp` clamps the old seq), so a mission after one that ended is "finished" on start and never takes off — item 0 is always flagged `current` on PX4; and PX4's MISSION_ITEM_REACHED carries the cursor after the no-position items that follow the waypoint (`set_mission_item_reached`), so a seq maps to the waypoint whose group `[wpSeq, next wpSeq)` contains it (ArduPilot reports the exact seq). Flown end to end on both: PX4 → ArduCopter task graph. **VTOL (PX4)**: the takeoff stays `NAV_TAKEOFF` (PX4 stays multicopter); the waypoint action `vtol_transition` (to FW / to MC) → `DO_VTOL_TRANSITION` after that waypoint; auto landing → `NAV_VTOL_LAND`. The device model's `vehicleType` reaches the encoder as `task.vehicle_type` (`sendToDevice` → encoder context → `missionTaskForDevice`), and a transition on anything that is not a VTOL is refused. PX4's VTOL airframes (SITL too) set `MIS_TKO_LAND_REQ=2`: a mission not ending in a landing is rejected, an RTL does not count. Flown in SITL (`gz_standard_vtol`) by `npm run test:e2e:vtol` (`test/e2e/test-vtol-mission-sitl.js`: needs the SITL on the ground, the server on :4000 and a `standard_vtol` device; sends ReturnHome/Land on failure and keeps the ULog). A fixed wing needs legs of hundreds of meters: a transition before a 10 m leg never completes.
- Tests: `test/test-mavlink.js` with real packets (`test/helpers/mavlinkFixtures.js`, no SITL), `test/test-device-protocol.js`.

## Element Type Catalog (`/api/markers/types`)

Sistema dinámico para registrar tipos de elementos de inspección (torres eléctricas, aerogeneradores, estructuras custom, etc.). Cada tipo puede tener un ícono 2D para el mapa y un modelo 3D para la vista de escena.

### Tipos estáticos (git-tracked)

Definidos en `server/config/planning/elementTypes.yaml`. Se distribuyen con el código y son comunes a todas las instalaciones:

```yaml
- id: windTurbine
  name: Wind Turbine
  icon: /api/markers/types/windTurbine/icon # URL servida por el servidor
  model3d: /api/markers/types/windTurbine/model
  height: 80
  color: green
  description: 'Aerogenerador'
```

Sus assets (ícono + modelo) van en `server/data/element-types/<id>/` y se trackean con git (excepto archivos >50MB).

### Tipos custom (runtime, no git-tracked)

Se guardan en `server/data/markerTypes.yaml` (ignorado por git). Los assets van en `server/data/element-types/custom_<timestamp>/`.

**Agregar un tipo custom via API:**

```bash
# 1. Crear el tipo
curl -X POST http://localhost:4000/api/markers/types \
  -H "Content-Type: application/json" \
  -d '{"name": "Mi Elemento", "description": "...", "height": 20, "color": "blue"}'
# → { "id": "custom_1712345678", ... }

# 2. Subir ícono (PNG/SVG/JPG)
curl -X POST http://localhost:4000/api/markers/types/custom_1712345678/icon \
  -F "file=@icono.svg"

# 3. Subir modelo 3D (GLB/GLTF) — opcional
curl -X POST http://localhost:4000/api/markers/types/custom_1712345678/model \
  -F "file=@modelo.glb"
```

**Agregar un tipo custom manualmente (desarrollo rápido):**

1. Poner los assets en `server/data/element-types/<id>/icon.svg` y `model.glb`
2. Agregar la entrada en `server/data/markerTypes.yaml`:

```yaml
- id: custom_123
  name: Mi Elemento
  description: Descripción
  height: 20
  color: blue
  icon: /api/markers/types/custom_123/icon
  model3d: /api/markers/types/custom_123/model # omitir si no hay modelo
```

### Endpoints disponibles

| Método | Ruta                           | Descripción                                |
| ------ | ------------------------------ | ------------------------------------------ |
| GET    | `/api/markers/types`           | Lista todos los tipos (estáticos + custom) |
| POST   | `/api/markers/types`           | Crea un tipo custom                        |
| DELETE | `/api/markers/types/:id`       | Elimina un tipo custom y sus assets        |
| GET    | `/api/markers/types/:id/icon`  | Sirve el ícono                             |
| POST   | `/api/markers/types/:id/icon`  | Sube/reemplaza el ícono                    |
| GET    | `/api/markers/types/:id/model` | Sirve el modelo 3D                         |
| POST   | `/api/markers/types/:id/model` | Sube/reemplaza el modelo 3D                |

### Archivos relevantes

- `server/models/markers/` — lógica de negocio del catálogo, separada por entidad: `elementTypes.js`, `elementGroups.js`, `elementItems.js`, `bases.js`, `assignments.js`, `inspectionTargets.js`, `markers.js` (barrel/orquestación)
- `server/controllers/markers.js` — handlers HTTP + configuración Multer
- `server/routes/markers.js` — definición de rutas
- `server/config/planning/elementTypes.yaml` — tipos estáticos
- `server/data/markerTypes.yaml` — tipos custom (runtime)
- `server/data/element-types/` — assets de todos los tipos

---

## Configuration Files

**Server Configuration** (`server/.env`, all read through `config/config.js` — that file is the SSOT, check it before assuming a var name):

- `PORT`: Server port (default 4000)
- `ROS_CONNECTION`: Enable/disable ROS bridge (default true)
- `FB_CONNECTION`: Enable/disable FlatBuffer protocol (default true)
- `MQTT_CONNECTION`: Enable the MQTT device protocol (default **false**, needs a broker) / `MQTT_URL`: broker URL (default `mqtt://127.0.0.1:1883`)
- `MAVLINK_CONNECTION`: Enable the MAVLink device protocol (default **false**) / `MAVLINK_LINKS`: comma-separated `udp:<bind host>:<port>` (listen) or `tcp:<host>:<port>` (connect), default `udp:0.0.0.0:14550`. Only one process can bind the UDP port (no `reuseAddr`: a second GCS would silently split the datagrams)
- `ROS_URL`: ROS bridge WS URL (default `ws://127.0.0.1:9090`)
- `DB` / `DB_TYPE` / `DB_HOST` / `DB_USER` / `DB_PASSWORD` / `DB_NAME` / `DB_PORT`: enable + connect to an external DB (unset `DB` → local SQLite)
- `DB_STORAGE`: local SQLite file (default `server/data/sequelize.sqlite`). `npm test` runs with `DB=false DB_STORAGE=:memory:` so tests never open — or run startup migrations on — the real database. Tests that need catalog data read the git-tracked files under `data/`, not DB rows.
- `STREAM_SERVER`: Enable/disable video streaming (MediaMTX)
- `LLM`: Enable/disable chat features
- `LLM_PROVIDER`: `openai` | `gemini` | `anthropic`/`claude` | `ollama` | `llamacpp` | `openai-compatible` (default `openai`)
- `LLM_MODEL`: optional model id override; empty uses the provider default
- `LLM_OLLAMA_BASE_URL` / `LLM_LLAMACPP_BASE_URL` / `LLM_COMPATIBLE_BASE_URL`: Chat Completions endpoint (`/v1` appended if missing). The endpoint comes from these and ONLY these — the `LLM_*_API_KEY` vars are tokens, left empty for local servers. A URL found in a key var aborts startup with a message telling you which var to move it to.
- **Ollama context window**: `num_ctx` CANNOT be set over the OpenAI-compatible endpoint — it is ignored silently. Set `OLLAMA_CONTEXT_LENGTH` on the Ollama server (or bake `PARAMETER num_ctx` into a Modelfile), otherwise long prompts are truncated with no error. The handler logs this warning on every boot.
- **llama.cpp**: start `llama-server` with `--jinja` so tool calling uses a tool-enabled chat template.
- `TOOL_APPROVAL_ENFORCE`: human-in-the-loop gate for flight tools. **Default ON** — set to `'false'` to disable (opt-out, so a missing var never silently removes the gate)
- `TOOL_APPROVAL_REQUIRED`: comma-separated tools that need approval (default `load_mission_to_uav,start_mission`)
- `TOOL_APPROVAL_TTL_MS` / `TOOL_APPROVAL_SWEEP_INTERVAL_MS`: how long an approval stays answerable (default 5 min) and how often expired ones are swept (default 30s)
- `MCP_ENABLE`: Enable/disable MCP server integration (default false)
- `MCP_CONFIG`: JSON configuration for MCP transport (`{"transport":"stdio","url":"http://localhost:3000/mcp"}`)
- `PLANNING_SERVER` / `PLANNING_HOST`: enable + point to the `mip_planner` FastAPI service (see root `CLAUDE.md`)
- `PROCESS_THERMAL_IMG`: enable thermal image post-processing pipeline
- `WS_PING_INTERVAL_MS` / `WS_POSITIONS_INTERVAL_MS` / `WS_STATE_INTERVAL_MS`: client WS timers (heartbeat, position-batch flush, server-state broadcast)
- `DEVICE_CHECK_INTERVAL_MS` / `DEVICE_UPDATE_INTERVAL_MS` / `DEVICE_TIMEOUT_MS`: device health-check cadence and OFFLINE threshold
- `TELEMETRY_CHECK_INTERVAL_MS` / `TELEMETRY_SLOT_TIMEOUT_MS`: per-slot telemetry watchdog tick (default 2s) and silence threshold (default 5s; per slot: `timeout_s`)
- `MAP_LATITUDE` / `MAP_LONGITUDE` / `MAP_ZOOM`: default map center for the local-origin ENU conversion

**Device Configuration** (`server/config/devices/devices_init.yaml`):

- Device registry on server startup
- Protocol, credentials, camera endpoints
- ROS topic mappings per device category

**MCP Server Configuration** (`mcp_server/.aitk/mcp.json`):

- MCP server metadata and configuration
- Debug ports and launch settings
- Tool definitions for AI integration

## Video Streaming

- Server: MediaMTX (configurable in `server/config/mediamtx.yml`)
- Protocols: WebRTC, RTSP, WebSocket
- Device camera array: Multiple cameras per UAV
- Client: Real-time video overlays on map

## Testing Strategy

- Server tests: Node.js built-in test runner (`npm test` → `node --test test/*.js`)
- API fixtures: `test/api/json/` holds real request/response payloads (mission plans, MCP tool calls, validation reports) used as golden files
- E2E: `test/e2e/` (run individually, e.g. `npm run test:e2e:collision` for the LLM collision-avoidance scenario)
- Integration tests: ROS message mocking, WebSocket simulation
- Frontend: Manual testing via development server (no client test script configured)

## Common Gotchas

1. **ROS Bridge Dependency**: Server requires ROS bridge running for ROS devices, even if not all devices use ROS
2. **Port Conflicts**: Frontend dev server (3000) and backend (4000) must not conflict. MCP server uses port 3001 for SSE debugging
3. **WebSocket Reconnection**: Clients auto-reconnect, but may miss events during disconnect
4. **State Machine Transitions**: Mission state changes require explicit state machine events
5. **Device Health Checks**: Devices marked OFFLINE after 30s without position updates
6. **Coordinate Systems**: three frames in play, don't mix them up — GeoJSON/map UI uses `[lon, lat]`; mission planning (`missionDataXYZ`, `mip_planner`) uses local ENU `[x, y, z]` meters; `missionEncodeConfig.js` then maps those XYZ values straight into fields literally named `latitude`/`longitude`/`altitude` for the ROS/PX4 bridge — the field names are misleading, they don't hold geodetic degrees
7. **Redux Migration**: Old planning format uses indexes, new format uses baseId references
8. **Git Submodules**: Remember to initialize/update submodules after cloning (`git submodule update --init --recursive`)
9. **MCP Server**: When developing MCP tools, the server auto-restarts may cause temporary disconnections (auto-reconnection handles this)
10. **Controller statics share one namespace**: controllers mix HTTP handlers (`(req, res)`) and facade methods for other models as `static` fields of the same class. A second `static` with the same name silently replaces the first — a facade `getMission(missionId)` once overrode the `GET /api/missions` handler and broke the mission list with no error. Give facades distinct names (`getMissionById`), and grep for the name before adding one.
11. **Tests and the real database**: always run tests through `npm test` (it sets `DB=false DB_STORAGE=:memory:`). A plain `node --test` of any file that imports `common/sequelize.js` opens — and runs startup migrations on — `data/sequelize.sqlite`. `--test-force-exit` is there because importing `models/devices.js` starts the `DeviceHealthMonitor` timers at import time, which would keep a test process alive.
12. **`positionsModel.getByDeviceId` is synchronous on purpose**: it only reads the in-memory cache, and `missionSignals` / `events.js` call it without `await`. While it was `async` they read a Promise, so the mission anomalies, `wp_reached` and the event position silently never worked. Keep it synchronous (`test/test-mission-signals.js`).
13. **Node 20 APIs only**: the server runs on Node 20 (`.nvmrc` v20.19, `npm test` uses it), so no
    `Map.groupBy` / `Object.groupBy`, `Set.prototype.union`/`intersection`, `Promise.withResolvers` or
    other Node 21+ builtins. `eslint-plugin-es-x` (`flat/restrict-to-es2023` in `eslint.config.js`) now
    catches these at lint time regardless of which code path exercises them — `engines` (`>=20`) alone
    never did, it's not enforced by anything.

## External Dependencies

- **ROS Noetic / ROS2**: For UAV communication (optional, based on protocol — see `models/ros/` for the ROS1→ROS2 split)
- **MIP Planner** (`mip_planner/`, sibling FastAPI service, port 8000): python-mip/CBC solver that replaces the LLM sub-agent for mission planning; enabled via `PLANNING_SERVER`/`PLANNING_HOST` (see root `CLAUDE.md`)
- **PostgreSQL/SQLite**: Device and mission persistence (optional, based on DB config)
- **MediaMTX**: Video streaming server
- **OpenStreetMap**: Map tiles (can be self-hosted for offline use)
- **Glyphserver**: Font rendering for maps (optional)
- **MCP Server** (submodule, `mcp_server/`): Model Context Protocol server for LLM tool integration (optional, based on LLM config)
- **LLM providers**: OpenAI, Google Gemini, Anthropic Claude, or any OpenAI-compatible server (Ollama, llama.cpp, vLLM, LM Studio…) — selected via `LLM_PROVIDER`
