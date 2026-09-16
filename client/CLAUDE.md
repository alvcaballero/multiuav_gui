# Client CLAUDE.md

## Directory Structure

```
client/
├── index.html              # Vite entry point
├── vite.config.js          # Vite configuration
├── package.json            # Dependencies (React 19, MUI v7, MapLibre)
├── .eslintrc.js            # ESLint configuration
├── build/                  # Production build output
├── public/                 # Static assets
└── src/
    ├── index.jsx           # React entry point
    ├── App.jsx             # Main app component with routing
    ├── AppThemeProvider.jsx # MUI theme configuration
    ├── Navigation.jsx      # Route definitions
    ├── ServerProvider.jsx  # Server initialization wrapper
    ├── SocketController.jsx # WebSocket lifecycle manager
    ├── reactHelper.js      # React utility functions
    ├── store/              # Redux state management
    ├── components/         # UI components grouped by domain
    │   ├── chat/           # ChatDrawer, ChatInput, ChatMessages
    │   ├── devices/        # DeviceList, DeviceRow, StatusCard, Adduav
    │   ├── mission/        # MissionPanel, MissionStats, MissionElevation, Routes, Waypoints
    │   ├── camera/         # CameraDevice, CameraV1, CameraWebRTCV4
    │   ├── map/            # ElementList, BaseList, BaseSettings
    │   ├── commands/       # CommandCard, SendCommand
    │   ├── layout/         # Navbar, MainToolbar, Menu, MenuItems, Footer
    │   └── ui/             # Toast, RemoveDialog, SelectList, SaveFile, PositionValue
    ├── pages/              # Page-level components (one per route)
    ├── settings/           # Settings pages
    ├── map/                # Map rendering (MapLibre GL)
    ├── scene3d/            # 3D visualization (React Three Fiber)
    ├── services/           # Business logic services
    ├── shared/             # Shared utilities, hooks, and base components
    │   ├── components/     # ErrorHandler, PageLayout, SelectField, SwipeConfirm, etc.
    │   ├── theme/          # MUI theme configuration
    │   ├── attributes/     # Attribute hooks
    │   └── util/           # Utility functions (duration, etc.)
    └── resources/          # Static resources image assets
        ├── images/             # Image assets
        ├── 3d/                 # 3D model assets
        └── lastimages/         # Recent images cache
```

## Architecture Overview

### State Management

**Client-side State (Redux):**

- `devices`: Device list, selection, follow mode
- `session`: Positions, camera feeds, markers, planning data, 3D scene
- `mission`: Mission editing (waypoints, routes, attributes)
- `chat`: LLM conversation histories
- `events`, `geofences`, `errors`: Supporting state slices

### Frontend Structure

**Key Components:**

- `client/src/map/`: Map rendering, device markers, route visualization
- `client/src/components/`: UI components (ChatDrawer, device panels)
- `client/src/store/`: Redux slices and selectors
- `client/src/SocketController.jsx`: WebSocket lifecycle manager
- `client/src/ServerProvider.jsx`: Server initialization wrapper

**Map Integration:**

- Uses MapLibre GL for rendering
- OpenStreetMap tiles (configurable for offline use)
- Real-time device position markers
- Live route history tracking
- Waypoint editing with drag-and-drop

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

## Known Issues Needing Architectural Decisions

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
(`MissionConvert.js`) and possibly server-side mission parsing, so it needs a decision on
migration strategy for existing saved missions before implementing.
