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
│   ├── chat/                    # LLM chat orchestrator, provider handlers, agent profiles (see below)
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
   - Decodes ROS messages via `models/ros/rosDecode.js` into internal position/status format
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

Key events: `MISSION_PLAN_SHOWN`, `POSITION_UPDATED`, `CAMERA_RECEIVED`, `DEVICE_UPDATED`, `CHAT_CREATED`

### Database Models

Location: `server/schemas/database/`

**Existing Models** (`schemas/database/index.js` is the barrel — register new ones there):

| Model            | Purpose                                                |
| ---------------- | ------------------------------------------------------- |
| User              | Auth/user registry                                     |
| Device            | UAV registry                                           |
| Mission           | Mission tracking                                       |
| MissionRoute      | Per-UAV route within a mission                         |
| MissionPlan       | LLM/MIP-planner-generated plan, pre-execution           |
| PositionHistory   | Sampled position history for the map's time slider      |
| File              | Downloaded mission artifacts (logs, images)             |
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
- Mission tracking: per-UAV XState actors (`models/mission/missionSM.js` spawns one `missionExecutionSM.js` machine per `uavId`, keyed in `listSM`)
- Position updates: In-memory cache with EventBus broadcast

**Redux Data Flow:**

```
WebSocket → SocketController.onmessage → dispatch(action) →
Reducer updates state → useSelector triggers re-render
```

### Mission Planning System

**Mission Structure** (`missionDataXYZ`, produced by the LLM planner and/or `mip_planner`):

```javascript
{
  route: [{
    uav: string,              // Device name, e.g. "px4_1"
    id: number,               // Route index
    uav_type: string,         // "px4_ros2", "px4_sitl", etc.
    attributes: {              // values + valid ranges come from config/devices/mission_schema.yaml (SSOT)
      max_vel: number,        // m/s
      idle_vel: number,       // m/s
      mode_yaw: 0-5,          // HEADING_MODE_* (0=Auto,1=Fixed,2=RC,3=Waypoint custom,4=POI,5=Gimbal yaw)
      mode_gimbal: 0-1,       // GIMBAL_MODE_* (0=Free,1=Waypoint)
      mode_trace: 0-3,        // FLIGHT_PATH_MODE_* (0=Curve,1=Curve+stop,2=Straight+stop,3=Coordinate turn)
      mode_landing: 0-4       // MISSION_FINISHED_* (0=No action,1=Home,2=Land,3=First WP,4=Infinite)
    },
    wp: [{
      type: string,           // "takeoff" | "inspection" | ... — first wp is usually "takeoff"
      pos: [x, y, z],         // LOCAL ENU meters, NOT lat/lon — z is altitude AGL
      yaw: -180 to 180,       // 0=North, 90=East, -90=West (optional)
      gimbal: number,         // Pitch angle in degrees (optional)
      speed: number,          // m/s (optional)
      notes: string,          // human-readable waypoint label (optional)
      action: {}              // Custom actions (optional)
    }]
  }]
}
```

Coordinates are XYZ local ENU end-to-end through planning (see `mip_planner/CLAUDE.md`
and `models/mission/coordinateConverter.js`). Conversion to/from geodetic
(`convertMissionXYZToLatLong`, `geodeticToENU`/`ENUToGeodetic`) happens only at the
edges — when talking to ROS/PX4 (`missionEncodeConfig.js` maps `pos[0..2]` straight
to `latitude/longitude/altitude` fields expected by the firmware bridge, so double-check
which coordinate frame a given consumer expects) or when rendering on the geo map.

**Mission Execution Flow:**

1. Plan mission in UI → Create mission via API
2. Load mission: `commandsController.sendCommandDevice('loadMission')` → ROS service call
3. Start mission: State machine transitions to RUNNING
4. Monitor execution: Position updates track waypoint progress
5. Complete mission: Files downloaded via FTP/SFTP
6. State machine: see `models/mission/missionExecutionSM.js` for the full XState definition (states include `LoadMission`, `Commadmission`, `RunningMission`, `UAVDownloadFiles`, `DownloadFilesGCS`, `resetUAV`, `return2home`, `stopMission`, `END` — not a simple linear pipeline)

**Command Execution:**

- Two channels: ROS services OR FlatBuffer commands
- 5-second timeout with automatic error callbacks
- Command types: `loadMission`, `commandMission`, `CameraFileDownload`, custom commands

### ROS Integration

**Device Configuration** (`server/config/devices/devices_msg.yaml`):
Maps device categories to ROS topics and message types:

```yaml
dji_M210_noetic:
  topics:
    position:
      name: '/dji_osdk_ros/gps_position'
      messageType: 'sensor_msgs/NavSatFix'
    battery:
      name: '/dji_osdk_ros/battery_state'
      messageType: 'sensor_msgs/BatteryState'
  services:
    configureMission:
      name: '/dji_control/configure_mission'
      serviceType: 'aerialcore_common/ConfigMission'
```

**Message Decoding** (`models/ros/rosDecode.js`):

- `sensor_msgs/NavSatFix` → GPS position
- `px4_msgs/VehicleStatus` → Arming state, navigation state, failsafe
- `sensor_msgs/Imu` → Heading from quaternion
- `sensor_msgs/BatteryState` → Battery percentage

**Message Encoding** (`models/ros/rosEncode.js`):

- Converts mission objects to ROS service request format
- Supports legacy `aerialcore_common/ConfigMission` and `px4_msgs/TrajectorySetpoint`

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
| `rosTopics.js`        | Topic transport primitives: `subscribeTopics`/`unsubscribeKey`/`unsubscribeAll`/`PubRosMsg`. Keeps `activeSubscriptions` (opaque subscriptionKey → listeners); no category/camera/topic-name knowledge                                        |
| `rosServices.js`      | ROS service calls: `callRosService` (primitive) + `callService` (receives the already-built service name + type; no config lookup)                                                                                                            |
| `rosAction.js`        | ROS2 action lifecycle + registry (see below). Device-layer methods receive the already-built `actionServerName`; no config lookup nor name-building                                                                                           |
| `rosInspect.js`       | Read-only rosapi introspection (topics/services/types/action servers)                                                                                                                                                                         |
| `rosDecode.js`        | ROS message → internal format                                                                                                                                                                                                                 |
| `rosEncode.js`        | Internal format → ROS service/message request                                                                                                                                                                                                 |
| `rosValidateMSG.js`   | Message structure validation against type maps                                                                                                                                                                                                |
| `ros2ActionClient.js` | Alternative action client (`callOnConnection`); currently unused                                                                                                                                                                              |
| `index.js`            | Module barrel exports                                                                                                                                                                                                                         |

**Config resolution lives ONLY in the facade.** `categoryModel` (in
`models/category.js`) is the single source of truth for `devices_msg.yaml` — it
is the ONLY module that reads that file. No file under `models/ros/` reads it.
The facade resolves per-category message config through one helper and builds
every ROS name through another:

- `resolveCategoryConfig(category, block, type)` → `categoryModel.getCategory(category)?.[block]` (with `type`, the single entry). `block` is `'subscribers' | 'publishers' | 'services' | 'actions'`. Using `categoryModel` (not a frozen `readDataFile` snapshot) means a runtime category edit is picked up immediately.
- `buildDeviceName(name, entry)` → `/${name}${entry.name}`. Device names are stored WITHOUT a leading slash and every config `name` starts with one, so every resulting topic/service/action name is absolute (`/agv_1/odom`). All four blocks use this one helper, so a device's topics, services and actions resolve identically.

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
those names for its non-HTTP callers (e.g. `commandsModel.standarCommand`); the
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
- **LLM Factory** (`handlers/llmFactory.js`, `LLMFactory`): picks a handler by `LLMProvider` (`config.js`) — `'openai'`, `'gemini'`, `'anthropic'`/`'claude'`, `'ollama'`
- **Provider handlers** (`handlers/*Handler.js`): `openaiHandler.js`, `geminiHandler.js`, `antropicHandler.js`, `ollamaHandler.js`, each extending `baseLLMhandler.js` — normalize that provider's tool-calling/streaming quirks behind one interface
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

When modifying mission flow, update both:

1. State machine definition (`models/mission/missionSM.js` / `missionExecutionSM.js`)
2. Command handlers in `models/commands.js`

### ROS Message Handling

When adding new device types:

1. Add configuration to `devices_msg.yaml` (`topics`, `services`, and/or `actions` per category)
2. Add decoder logic in `models/ros/rosDecode.js`
3. Add encoder logic in `models/ros/rosEncode.js` (if sending commands)
4. Subscriptions are created automatically — the facade (`ros.js`) resolves the config via `categoryModel` and hands built topics to `models/ros/rosTopics.js`; no per-device code needed

### WebSocket Message Format

```javascript
// Server → Client
wsController.sendMessage({ type: 'positions', positions: {...} });

// Client processing (SocketController.jsx)
dispatch(sessionActions.updatePositions(data.positions));
```

## Device Communication Protocols

**ROS Protocol** (`protocol: 'ros'`):

- Requires ROS bridge running on `ws://127.0.0.1:9090`
- Launch: `roslaunch aerialcore_gui connect_uas.launch`
- Topics auto-subscribed based on device category

**FlatBuffer Protocol** (`protocol: 'robofleet'`):

- Direct WebSocket connection to fleet
- Binary encoding for bandwidth efficiency
- Used for non-ROS devices or optimized deployments

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
- `ROS_URL`: ROS bridge WS URL (default `ws://127.0.0.1:9090`)
- `DB` / `DB_TYPE` / `DB_HOST` / `DB_USER` / `DB_PASSWORD` / `DB_NAME` / `DB_PORT`: enable + connect to an external DB (unset `DB` → local SQLite)
- `STREAM_SERVER`: Enable/disable video streaming (MediaMTX)
- `LLM`: Enable/disable chat features
- `LLM_PROVIDER`: `openai` | `gemini` | `anthropic`/`claude` | `ollama` (default `openai`)
- `TOOL_APPROVAL_ENFORCE`: human-in-the-loop gate for flight tools. **Default ON** — set to `'false'` to disable (opt-out, so a missing var never silently removes the gate)
- `TOOL_APPROVAL_REQUIRED`: comma-separated tools that need approval (default `load_mission_to_uav,start_mission`)
- `TOOL_APPROVAL_TTL_MS` / `TOOL_APPROVAL_SWEEP_INTERVAL_MS`: how long an approval stays answerable (default 5 min) and how often expired ones are swept (default 30s)
- `MCP_ENABLE`: Enable/disable MCP server integration (default false)
- `MCP_CONFIG`: JSON configuration for MCP transport (`{"transport":"stdio","url":"http://localhost:3000/mcp"}`)
- `PLANNING_SERVER` / `PLANNING_HOST`: enable + point to the `mip_planner` FastAPI service (see root `CLAUDE.md`)
- `PROCESS_THERMAL_IMG`: enable thermal image post-processing pipeline
- `WS_PING_INTERVAL_MS` / `WS_POSITIONS_INTERVAL_MS` / `WS_STATE_INTERVAL_MS`: client WS timers (heartbeat, position-batch flush, server-state broadcast)
- `DEVICE_CHECK_INTERVAL_MS` / `DEVICE_UPDATE_INTERVAL_MS` / `DEVICE_TIMEOUT_MS`: device health-check cadence and OFFLINE threshold
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

## External Dependencies

- **ROS Noetic / ROS2**: For UAV communication (optional, based on protocol — see `models/ros/` for the ROS1→ROS2 split)
- **MIP Planner** (`mip_planner/`, sibling FastAPI service, port 8000): python-mip/CBC solver that replaces the LLM sub-agent for mission planning; enabled via `PLANNING_SERVER`/`PLANNING_HOST` (see root `CLAUDE.md`)
- **PostgreSQL/SQLite**: Device and mission persistence (optional, based on DB config)
- **MediaMTX**: Video streaming server
- **OpenStreetMap**: Map tiles (can be self-hosted for offline use)
- **Glyphserver**: Font rendering for maps (optional)
- **MCP Server** (submodule, `mcp_server/`): Model Context Protocol server for LLM tool integration (optional, based on LLM config)
- **LLM providers**: OpenAI, Google Gemini, Anthropic Claude, or a local Ollama instance — selected via `LLM_PROVIDER`
