# Client CLAUDE.md

## Directory Structure

```
client/
├── index.html              # Vite entry point
├── vite.config.js          # Vite configuration
├── package.json            # Dependencies (React 19, MUI v7, MapLibre)
├── eslint.config.js        # ESLint configuration
├── build/                  # Production build output
├── public/                 # Static assets
└── src/
    ├── index.jsx           # React entry point
    ├── App.jsx             # Main app component with routing
    ├── AppThemeProvider.jsx # MUI theme configuration
    ├── Navigation.jsx      # Route definitions (lazy-loaded pages)
    ├── ServerProvider.jsx  # Server initialization wrapper
    ├── SocketController.jsx # WebSocket lifecycle manager
    ├── ErrorBoundary.jsx   # Top-level error boundary
    ├── reactHelper.js      # React utility functions (useCatch, useAsyncTask, ...)
    ├── store/              # Redux state management
    ├── pages/              # One file per main route; helpers live in feature subfolders
    │   ├── MainPage / MainPage3D, MissionPage / MissionPage3D, PlanningPage,
    │   │   DevicePage / DevicePage3D, CameraPage, ChatPage, Scene3DEditorPage
    │   ├── planning/       # Hooks used only by PlanningPage (actions, redux handlers, polling)
    │   ├── device/         # DevicePage panels (header, stage, mission, telemetry, telemetry log,
    │   │                   #   manual control) and its hooks (capabilities, commands, telemetry log, keys).
    │   │                   #   The telemetry log shows the DECODED position diffs, not raw messages
    │   ├── deviceMessages/ # DeviceMessagesPage (/device/:id/messages): the raw messages of one slot of a
    │   │                   #   device as they reached the server (before decode), as a JSON tree. Asks for
    │   │                   #   them over the shared WebSocket (raw:subscribe / raw:unsubscribe, see
    │   │                   #   server/CLAUDE.md); the messages stay out of Redux (up to 10 per second).
    │   │                   #   The first one may be the server's stored copy (`cached`, `ageMs`). The
    │   │                   #   list shows each slot's rate/health from GET /api/devices/:id/telemetry
    │   ├── reports/        # EventsPage, Mission*Report*Page, missionDetailReport/ (its sections)
    │   └── other/          # GeofencesPage (+GeofencesList), ReplayPage, TopicsPage (+SquareMove), MissionPageTest
    ├── components/         # Feature components, grouped by domain
    │   ├── chat/           # ChatDrawer, ChatPanel, ChatThread, ApprovalCard, ToolCallRow, ChatInput, useChatLogic, chatTheme
    │   ├── devices/        # DeviceList, DeviceRow, StatusCard, AddDevice (dialog), DeviceCameraEditor, DeviceFilesEditor
    │   │                   #   (each entry a DeviceSourceCard), useFilter,
    │   │                   #   DeviceIdentityEditor (name → protocol → category → the protocol's attributes, shared by
    │   │                   #   AddDevice and settings/SettingsDevicesPageEdit; its rule in deviceIdentity.js mirrors the
    │   │                   #   server's). The protocol comes first: GET /api/server/protocol lists the four with
    │   │                   #   `enabled` (disabled ones greyed out) and the categories come from
    │   │                   #   GET /api/category?protocol= (see server/CLAUDE.md › Protocol vs message format).
    │   │                   #   A device has no `ip`: a camera's address is `camera[].host`, in DeviceCameraEditor
    │   │                   #   (required unless Websocket). See "Device attributes" below
    │   ├── mission/        # MissionPanel, MissionStats, MissionElevation, Route/Waypoint lists, tracking
    │   ├── camera/         # CameraDevice, CameraDevicePanel, CameraMediaMTXPlayer, SocketCameraCanvas
    │   ├── commands/       # CommandCard (send-command dialog). Labels, params, confirm and severity of
    │   │                   #   each command come from GET /api/commands/types (server command_schema.yaml)
    │   ├── layout/         # Navbar, Navbar2, MainToolbar, Menu, MenuItems, menu/ (popovers)
    │   └── planning/       # PlanningToolbar, ElementsTab, PlanningTab, SettingsTab, BaseList, ElementList
    ├── settings/           # Settings pages (devices, categories, element types, geofence)
    ├── map/                # Map rendering (MapLibre GL): controls, core, devices, draw, environment, mission, transform
    ├── scene3d/            # 3D visualization (React Three Fiber)
    ├── common/             # Shared by several features — same idea as traccar-web's `common/`
    │   ├── components/     # Generic UI: PageLayout, ErrorHandler, SelectField (with emptyValue={null} it
    │   │                   #   auto-selects the first item passing the whole ITEM, not its key, and keeps its old
    │   │                   #   list until a new endpoint answers: remount it with a `key` when the list changes;
    │   │                   #   `itemDisabled` greys items out), RemoveDialog, SelectList,
    │   │                   #   SaveFile, PositionValue, SwipeConfirm (see note below), form fields, and
    │   │                   #   ObstacleAvoidanceSensors (top-down view of obstacle_info; helpers
    │   │                   #   in util/obstacles.js)
    │   ├── theme/          # MUI theme (palette, dimensions, component overrides)
    │   └── util/           # formatter, converter, fetchs, missionConvert, missionGeo, preferences,
    │       │               #   commandTypes (mirror of the server CommandType; never which ones a
    │       │               #   device has — that's /api/commands/types),
    │       │               #   vehicleState (mirror of the server ARM_STATE / FLIGHT_PHASE / FLIGHT_MODE
    │       │               #   plus the text to show for each — see below),
    │       │               #   usePersistedState/useQuery/useMarkerTypes/useMissionFile, and the
    │       │               #   former services (planningService, fileService, missionPlanLoader,
    │       │               #   sendChatMessage, toolApproval)
    │       └── styles/     # Domain color maps: eventStyle, missionStatus, routeColors
    └── resources/          # Static resources image assets
        ├── images/             # Image assets
        ├── 3d/                 # 3D model assets
        └── lastimages/         # Recent images cache
```

### Where does a new file go?

- **A route** → `pages/` (root if it is a main page, `reports/` for report views, `other/` for the rest).
  A hook or subcomponent used by ONE page lives next to it (`pages/planning/`, `pages/reports/missionDetailReport/`).
- **A component for one feature** (chat, mission, planning, devices, ...) → `components/<feature>/`.
- **Something used by several features** → `common/`: UI in `components/`, pure helpers and hooks in
  `util/`, domain color maps in `util/styles/`, MUI theme config in `theme/`.
- **Map-only code** (layers, GeoJSON features, popups) → `map/`. Mission *data* conversion is not map code:
  it lives in `common/util/missionConvert.js`.
- Keep `common/` free of imports from `pages/` and `components/`. A few `util/` files already import
  `store/` (and `missionPlanLoader` imports `map/core/mapInstance`); do not add more.
- The UI has no i18n: write plain English text, never traccar-style keys (`sharedName`, `positionIgnition`, ...).

## Architecture Overview

### State Management

**Client-side State (Redux):**

- `devices`: Device list, selection, follow mode
- `session`: Positions, camera feeds, markers, planning data, 3D scene, and the server connections the
  toolbar chips show (`serverROS`; `serverMQTT` is `null` when the server does not run MQTT — no chip)
- `mission`: Mission editing — the plan being edited, as routes (each route = one task of the server's task graph, see below)
- `activeMissions`: Live tracking of missions and their tasks, fed by `GET /api/missions/tasks` + WS `taskUpdated`
- `chat`: LLM conversation histories
- `events`, `geofences`, `errors`: Supporting state slices

### Frontend Structure

**Key Components:**

- `client/src/map/`: Map rendering, device markers, route visualization
- `client/src/components/`: feature components (ChatDrawer, device panels); generic UI lives in `client/src/common/`
- `client/src/store/`: Redux slices and selectors
- `client/src/SocketController.jsx`: WebSocket lifecycle manager
- `client/src/ServerProvider.jsx`: Server initialization wrapper

**Map Integration:**

- Uses MapLibre GL for rendering
- OpenStreetMap tiles (configurable for offline use)
- Real-time device position markers
- Live route history tracking
- Waypoint editing with drag-and-drop

**Origin: traccar-web.** The map layer (`client/src/map/`) is based on
[traccar-web](https://github.com/traccar/traccar-web) (`src/map/main/`, `src/map/core/`). That is why
layers carry `metadata['traccar:title']` and why components follow its pattern: a component adds its
own source + layer in a `useEffect` (cleanup removes both), and a second `useEffect` pushes data with
`map.getSource(id).setData(...)`. When you need a new map layer, check whether traccar-web already has
one (e.g. `MapAccuracy`, `MapRoutes`, `MapGeofence`) and port it instead of inventing a new pattern.
Adaptations vs. upstream: `map` comes from `core/mapInstance`, no `toMapCoordinates`/`useTranslation`.

**Collision sensor arcs (`map/devices/MapCollisionSensor.js`):** draws one arc per side
(front/right/back/left) around the UAV from `position.attributes.obstacle_info`
(`[down, front, right, back, left, up]`, meters, 100 = no reading) rotated by `position.course`.
Closer obstacle → arc closer to the icon, more opaque, green→yellow→red; beyond `MAX_RANGE` nothing
is drawn. Radius is in screen pixels (recomputed on `zoom`), not real meters. `mode="selected"`
(default) draws only the selected device, `mode="all"` draws every UAV with a position.
Its colours are `HAZARD` (`common/theme/palette.js`), the same scale as the sensor widget, the device page alert
and the exclusion zones (`MapObstacles`): `obstacleColor(d)` for a fill, `obstacleTextColor(d)` (`HAZARD_TEXT`) for
a distance written on a light background — amber text on white is unreadable. It is a scale of its own, not
`STATUS`: "clear" is not "finished well".

**Layer visibility toggling (`traccar:title` metadata):**

`client/src/map/controls/MapSwitcher.jsx` builds its layer-visibility menu by scanning
`map.getStyle().layers` for a `metadata['traccar:title']` string on each MapLibre layer, grouping
layers that share the same title, and toggling their `visibility` layout property together. It
re-scans on every `styledata` event, so newly added layers are picked up automatically.

To make a layer group togglable from MapSwitcher, add matching metadata to every `map.addLayer()`
call that belongs to that group:

```javascript
map.addLayer({
  id: 'geofences-fill',
  type: 'fill',
  metadata: { 'traccar:title': 'Geofences' },
  // ...
});
```

All layers sharing the same title are shown/hidden as one unit — tag every layer in the group, not
just one, or part of it will stay stuck visible. Existing examples: `map/environment/MapGeofence.js`
(`'Geofences'`) and `map/environment/MapObstacles.js` (`'ObstaclesRegions'`).

**3D view controls & layers (`client/src/scene3d/`):**

The 3D view (React Three Fiber) has no MapLibre `map` object to call `addControl` on, so
`client/src/scene3d/controls/registry/` implements the same *convention* in plain React:

- `Scene3DControlProvider` renders one absolutely-positioned `<div>` per corner
  (`top-left`/`top-right`/`bottom-left`/`bottom-right`) that stacks its children via flexbox —
  the 3D equivalent of MapLibre's per-corner control containers.
- `<Scene3DControl corner="top-right">...</Scene3DControl>` portals its children into that
  corner's container. Wrap any new control's JSX in it instead of hand-picking a
  `position: absolute; top: <px>` offset — it stacks automatically below the existing controls
  in that corner. See `Scene3DNavigationControl.jsx`, `DownloadYamlButton.jsx`,
  `ScreenshotButton.jsx` for examples.
- `Scene3DControlProvider` must wrap the `<R3FCanvas>` (and its sibling control components) in
  `Scene3DCanvas.jsx`; controls themselves render as normal React siblings, not inside the canvas.

Layer visibility toggling is the `scene3d` equivalent of `traccar:title`, implemented in
`client/src/scene3d/layers/`:

- `Scene3DLayerProvider` holds the set of currently mounted layer titles and which are hidden
  (persisted via `usePersistedState('hiddenScene3DLayers', [])`).
- `useSceneLayer('My Title')` self-registers a scene component under that title and returns
  whether it should currently render. Multiple components can share a title to toggle together.
- `Scene3DLayerSwitcher.jsx` (a `Scene3DControl`) lists every registered title in a menu with a
  `Switch`, mirroring `MapSwitcher.jsx`.

To add a new toggle: call `useSceneLayer('Title')` in the component and use its return value to
mount conditionally — no changes needed in `Scene3DLayerSwitcher.jsx`, it picks up new titles
automatically. `client/src/scene3d/controls/OrientationGizmo.jsx` is intentionally NOT part of
this system — it renders directly into the WebGL canvas via `gl.setScissor`/`setViewport` inside
`useFrame`, not as an HTML overlay, so it can't be portaled like the other controls.

### Missions as task graphs

The server runs a mission as a graph of tasks (one device per task, `depends_on` between
them — see `server/CLAUDE.md` › Mission Planning System). The client side:

- **Editor (`/mission`, `store/mission.js`, `components/mission/RouteRouteList.jsx`)** still
  edits `route[]`, but every route carries its task fields: `task_id`, `depends_on`, `action`
  (`params` are the existing "Route Attributes"). Rules that keep it consistent:
  - `task_id` is assigned **once** and never derived from the list position (`nextTaskId`:
    first free `T<n>`), both in `addRoute` and when a plan is loaded (`RuteConvert`). A
    position-derived id would rename a task — and break every `depends_on` pointing at
    it — whenever an earlier route is deleted.
  - `deleteRoute` removes the deleted `task_id` from every other route's `depends_on`.
    A dangling dependency makes the server reject the whole plan.
  - The "Depends on" field only offers the OTHER routes. Everything else about the graph
    (cycles, two unordered tasks on one UAV) is validated by the server on load; its `400`
    message is shown to the user — don't duplicate those rules here.
- **Wire format:** `commandLoadMission` always sends `{ tasks }` built by `routesToTasks`
  (`common/util/missionConvert.js`); `RuteConvert` reads `tasks[]` (v4), `route[]` (v3) and legacy.
- **Tracking (`store/activeMissions.js`)**: tasks are keyed by **task id**, never by
  `deviceId` — a device can run several tasks of one mission. `upsertTask` merges: the
  diagnostics-only updates from the server's waypoint tracking carry no
  `taskKey`/`action`/`dependsOn`, and replacing the task would erase them.
- Task status styles live in `common/util/styles/missionStatus.js` (`taskStyle`); `init` is shown as
  "Waiting" (for its dependencies), `skipped` means it never ran because a dependency failed.
- Their colours, and the events', come from `STATUS` in `common/theme/palette.js` (also `palette.status` in `sx`),
  which fixes what a colour MEANS across the app: blue in progress, green finished well, amber needs the operator
  or went partly wrong, red failed, grey not started, blue-grey stopped. The assistant panel follows the same rule
  with its own shades (`StatusTag` tones are named like `STATUS`). Never pick a status colour in a component,
  nor the text on it: `missionStyle`/`taskStyle` return the chip's `sx` (spread it, don't rebuild it from `color`)
  and `textColor`, and events have `eventTextColor`, all from `contrastText()` (white on the solid shades, dark text
  on the light ones — all at WCAG AA). "Downloaded" is `outlined` (no fill, inset outline) so it does not read as a
  second "Done"; a status built outside the maps (the anomaly chip) goes through `statusStyle()` too.

### Vehicle state (`position.attributes`)

The server sends the vehicle state already canonical, as three independent axes — never a firmware
label (see `server/CLAUDE.md` › Message Decoding):

- `armState`: `ARMED | DISARMED | UNKNOWN`
- `flightPhase`: where it is — `ON_GROUND | TAKEOFF | IN_AIR | LANDING | UNKNOWN`
- `flightMode`: how it is flown — `MANUAL | ASSISTED | HOLD | MISSION | RTL | OFFBOARD | …`

`common/util/vehicleState.js` mirrors those constants (`server/test/test-vehicle-state-mirror.js` fails
if they drift) and owns the text to show: `armLabel` / `phaseLabel` / `modeLabel`, which return `null`
for `UNKNOWN` or an absent attribute — the caller picks a dash (stats) or nothing (captions). Compare
against the constants (`flightMode === FLIGHT_MODE.RTL`), never against a string or the label. The
old `navState` and `landed_state` no longer exist.

### Device attributes (`device.attributes`)

A device's own settings that depend on something else about it — its protocol, its category, or another
attribute — live in `device.attributes` (JSON), not in columns. Today the only one is mavlink's `{ systemId }`
(1..254). The server declares them as groups in `ATTRIBUTE_GROUPS` (`server/schemas/zod/devices.js`; full rules in
server/CLAUDE.md › Per-device attributes): strict, so an unknown key is a 400, and on `PUT` it drops the stored keys
no group applies to any more.

The client does NOT read those groups: it mirrors them by hand in `components/devices/` —
`DeviceIdentityEditor` shows the field (`isMavlink(protocol)` → system id), `deviceIdentity.js` validates it
(`systemIdError`, `isDeviceIdentityValid`) and `withProtocol()` resets `attributes` to `{}` when the protocol
changes (`{}`, not `undefined`, so an edit clears them in the database too). **A new group on the server needs
its field and rule added here**, or the form never sends it (or sends a key the server rejects). The server stays
the authority; the client only avoids sending what it would refuse.

### Colours (`common/theme/palette.js`)

A component never writes a hex, an `rgba()` or a `@mui/material/colors` shade: it asks the theme. In `sx` by
path (`'grey.300'`, `'toolbar.badge'`; a border is `border: 1, borderColor: '<path>'`, a path inside a template
string is not resolved), in `makeStyles` via `theme.palette.…`. Tokens are named by what they are for:

- **Meaning** — `STATUS` (state of a mission/task/event/connection), `HAZARD` / `HAZARD_TEXT` (how close a danger
  is), `GEOMETRY` (drawn shapes). Exported as constants too, for MapLibre / Three.js, which do not read the theme.
- **Chrome** — `navbar`, `toolbar`, `missionControl`, `panel`, `assignedDevice`; MUI's own `grey`, `text`,
  `divider`, `background` for the rest. `LOADER` is a constant because the page loader is inline CSS.
- **Text on a colour** goes through `contrastText()`; never fix it to white.
- **Data colours** are not theme: route colours live in `common/util/styles/routeColors.js`, and the same entity is
  drawn with the same colour in 2D and 3D (a 3D waypoint falls back to `NO_ROUTE_COLOR`, like the map).

Left as literals on purpose: the base map style (`map/core/martinStyle.js`), the 3D world (`R3FCanvas`,
`textures.js`, the gizmo axes), mapbox-gl-draw's own theme, the map and 3D controls (they copy MapLibre's native
look), video backgrounds and the HUD over video, and colours stored as data (an element type's default colour).

### Redux State Updates

- Use immer-style mutations in Redux Toolkit reducers
- Normalize data: Use maps `{ [id]: object }` for fast lookups
- Use selectors for complex queries (`store/sessionSelectors.js`)

### Migration Helpers

When changing session state structure, add migration in `store/sessionMigration.js`:

```javascript
export const migrateNewFeature = (oldState) => {
  // Transform old format to new format
  return newState;
};
```

## Notes

### `SwipeConfirm` scope — not a chat/agent confirmation gate

`common/components/SwipeConfirm.jsx` is used ONLY for manual, human-initiated actions
from the UI (loading a mission, in `Menu.jsx`/`MainPage.jsx`/`MainPage3D.jsx`). It has
no relation to the LLM chat.

The chat agents have their own, separate gate: `components/chat/ApprovalCard.jsx`.
Tools listed in `TOOL_APPROVAL_REQUIRED` (default `load_mission_to_uav`, `start_mission`)
park the server-side tool loop until an operator answers. Two rules that are load-bearing
rather than cosmetic:

- **A chat message never approves anything.** Only the structured response keyed by
  `requestId` (`common/util/toolApproval.js`) decides. With several requests open there is
  no way to tell which one free text meant, and guessing wrong arms an aircraft.
- **What is shown is what executes.** The gate renders the frozen input verbatim; the
  server re-checks its `sha256` before running it and refuses on any mismatch. The turn
  is never resumed by re-asking the model what it wanted.

Pending approvals live in the server DB (`ChatToolApproval`), so they survive a reload —
`pendingApprovals` in the Redux chat slice is a projection of that, never the source of
truth. Enforcement is behind `TOOL_APPROVAL_ENFORCE`.

The same card serves the eve engine (`request.source === 'eve'`), so it renders a request without knowing
which engine sent it: the answers come from `request.options` (legacy `approve`/`deny`, eve `approve`/`cancel`
or `continue`/`stop` for a session limit), and `expiresAt` / `inputHash` are optional (eve has neither). What
decides how a request looks lives in `common/util/inputRequest.js`, tested from `server/test/test-input-request-client.js`
because the client has no runner. Two differences that are on purpose: a legacy card leaves when the operator
clicks, an **eve card leaves when eve confirms** (`resolved`), because if the server cannot record or deliver the
answer it says so (`approvalFailures`, `CHAT_TOOL_APPROVAL_FAILED`) and the card must still be there; and the
turn's activity (`Thinking…` / `Working on <task>…`, `state.chat.activity`) is only reported by eve. Typing in the
chat while an eve approval is open CANCELS it (eve outcome `ignored`); it never approves.

What answers the open chat is shown in the composer and in the assistant label of the thread (`agentInfo` in
`useChatLogic`): `eve · <agent>`, or `<agent profile> · <provider> · <model>` for the legacy engine. `chat.agent`
is the eve agent or the legacy profile (`agents/*.md`, `default` when none was chosen); the server state's
`llmModel` is the model really in use, which is the handler's default when `LLM_MODEL` is empty.
It is a label, not a status: nothing is probed, and it is not a selector (a chat's agent is fixed when it is
created). A connection problem shows up when a message is sent, as an error in the chat (`explainEveError` for
eve: "The eve agent … is not reachable. Is it running?").

### The assistant panel (design handoff "AI Assistant Panel")

The panel keeps the handoff's layout and density, but **looks like the app**: it has no colour and no font of its
own. `chatTheme.js` is given to the panel through a nested `ThemeProvider` (in `ChatPanel`, so the docked drawer and
`/chat` both get it). It is built from the app theme's options (`themeOptions` in `common/theme`) — a nested
provider given an object REPLACES the outer theme, so `dimensions` and the app palette exist inside the panel only
because of that. The font is the app's system stack (`FONT_FAMILY`); no web font is loaded.

What it adds is vocabulary, every value **derived** from the app with `lighten`/`darken` (`scale()`, `tones()`):
a scale for `primary` (`primary.100`…`800`), and for each state a `tint`, `tag`, `border`, `rowBorder` and `text`.
The meaning is the app's: `primary` (the app's primary) is the operator — their messages, the main actions;
`running` (`STATUS.running`) is a tool call or subagent in progress, the same blue as a running task; `success`
(`STATUS.done`) approved / done; `warning` (`STATUS.attention`) waiting on the operator; `error` (`STATUS.failed`)
rejected or failed. Only `subagent` (MUI's teal, a finished subagent) has no app counterpart. `titleBar` is the
app's toolbar colours. Red or amber written on a tint uses `error.text` / `warning.text` (darkened): the plain
colour does not reach AA there. What the panel keeps of its own is density: 34px buttons without capitals, 22px
chips, smaller body text.

Components ask for colours by path in `sx` (`'warning.tint'`, `'primary.700'`, `'running.main'`) — no hex. A
border is `border: 1, borderColor: '<path>'`: a path inside a template string is not resolved. In `sx`, `width: 1`
is 100%, not 1px — write `'1px'`.

- **Docked, as tall as the map**: `ChatDrawer` starts under the navbar (`theme.dimensions.navbarHeight`); only its
  width is resizable.
- **Two ways to start a chat, one per engine**: a button for legacy (`+`) and one for eve (sparkles, only when
  `eveEnabled`). The engine is chosen by the click, never by a setting.
- **History** replaces the thread (`view` in `useChatLogic`): top-level chats only, grouped by day
  (`common/util/chatHistory.js`). `listChats` gives it `preview`, `subagentCount` and `parentChatId`.
- **Subagents** are chats with `metadata.parentChatId`. The title-bar switcher lists them (the DB list plus the
  server's in-memory registry, which knows which are running); opening one is just `handleSelectChat`. Rows reach
  that action through `ChatNavigationContext`, not props, so the `memo` of the transcript rows survives.
- **A tool call and its result are one row** (`groupTranscript` in `common/util/chatMessage.js`, paired by
  `call_id`). Collapsed rows keep `content-visibility: auto`.
- **Approval card**: Approve / Reject / "Do this instead…". The last is a rejection that carries the operator's text
  (`denyReason` for legacy, the option's `text` for eve): the command is never sent. Decided requests stay in the
  thread as a line (`resolvedApprovals`, session-only; `resolvedOutcome` decides what it says).

Pure logic (`chatMessage.js`, `chatHistory.js`, `inputRequest.js`) is tested from `server/test`
(`test-chat-transcript.js`, `test-input-request-client.js`).

Design rationale and the rest of the protocol work this belongs to: `docs/ChatProtocol.md`.

## Known Issues Needing Architectural Decisions

### Chat message contract (no stable id, three clocks, raw-JSON fallback)

The chat wire format has no stable message id (the DB PK never leaves the backend), three
different clocks stamp the same message's timestamp, images travel and are stored as inline
base64, and an unrecognised message `type` falls through `convertMsg`
(`convertMsg` in `common/util/chatMessage.js`) into a plain text bubble via `ReactMarkdown` —
so a client-side artifact is visually indistinguishable from a real assistant message, and
markdown-significant characters in the payload corrupt what is shown.

These are symptoms of one root cause: `loadHistory` (REST) and the WebSocket emit are two
separate pipelines producing different shapes.

Full diagnosis, the proposed event-based contract, and the flight-safety requirements for
human-in-the-loop tool approval are in `docs/ChatProtocol.md`. That document is **proposed,
not accepted** — it needs a decision before implementation, since it changes the chat
persistence schema.

### Missing stable IDs on mission data (waypoints, bases, elements)

`react-doctor`'s `no-array-index-as-key` flags array index as `key` in three editable list components:

- `src/components/mission/RouteRouteList.jsx:266` — waypoints (`route.wp`)
- `src/components/planning/BaseList.jsx:135` — bases (`markers`)
- `src/components/planning/ElementList.jsx:115` — elements (`markers`)

All three share the same root cause: the underlying data shape (`{ pos: [...] }` for waypoints,
`{ latitude, longitude, name, ... }` for bases/elements) has no `id` field, yet the reducers
genuinely `splice()` these arrays at runtime (`missionActions.addWaypoint` with `insertAt`,
`deleteWaypoint`, `DeleteElement`, etc.) — not just append/clear. Using the array index as `key`
means inserting or deleting a mid-list item can reassign React's DOM/state to the wrong row
(an open accordion or in-progress edit jumping to a different waypoint/base).

The correct fix is adding a stable id (e.g. `crypto.randomUUID()`) at creation time in the
relevant Redux reducers (`store/mission.js` — `addWaypoint`, and the equivalent base/element
creation actions), then keying off `waypoint.id` / `base.id` instead of the array index. This was
deferred because it changes the mission data schema — it likely touches mission save/load format
(`common/util/missionConvert.js`) and possibly server-side mission parsing, so it needs a decision on
migration strategy for existing saved missions before implementing.
