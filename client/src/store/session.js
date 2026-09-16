import { createSlice } from '@reduxjs/toolkit';
import { migrateMarkers, migratePlanning, generateLocalId } from './sessionMigration';

const { reducer, actions } = createSlice({
  name: 'session',
  initialState: {
    server: null,
    serverROS: null,
    user: {},
    socket: null,
    includeLogs: false,
    logs: [],
    positions: {},
    history: {},
    markers: {
      bases: [],
      elements: [],
    },
    planning: {
      id: null,
      objetivo: { id: 1 },
      loc: [],
      meteo: [],
      assignments: [],
      defaultSettings: {},
      settingsSchema: {},
    },
    scene3d: {
      origin: { lat: 37.410381, lng: -6.002094, alt: 400 },
      range: 1000,
    },
  },
  reducers: {
    updateServer(state, action) {
      state.server = action.payload;
    },
    updateServerROS(state, action) {
      //state.server.rosState = action.payload;
      state.serverROS = action.payload;
    },
    updateUser(state, action) {
      state.user = action.payload;
    },
    updateSocket(state, action) {
      state.socket = action.payload;
    },
    updatePositions(state, action) {
      //console.log('Updating positions', action.payload);
      const liveRoutes =
        state.user?.attributes?.mapLiveRoutes || state.server.attributes.mapLiveRoutes || 'none';
      const liveRoutesLimit =
        (state.user?.attributes && state.user?.attributes['web.liveRouteLength']) ||
        state.server.attributes['web.liveRouteLength'] ||
        10;
      action.payload.forEach((position) => {
        state.positions[position.deviceId] = position;
        if (liveRoutes !== 'none') {
          const route = state.history[position.deviceId] || [];
          const last = route.at(-1);
          if (!last || (last[0] !== position.longitude && last[1] !== position.latitude)) {
            state.history[position.deviceId] = [
              ...route.slice(1 - liveRoutesLimit),
              [position.longitude, position.latitude],
            ];
          }
        } else {
          state.history = {};
        }
      });
    },
    updateMarker(state, action) {
      if (action.payload && action.payload.elements) {
        // Migrar markers automáticamente si vienen sin IDs
        const migrated = migrateMarkers(action.payload);
        state.markers = migrated;
      }
    },
    addMarkerElement(state, action) {
      state.markers.elements.push(...action.payload);
    },
    addMarkerBase(state, action) {
      // Las bases importadas (ej. desde KML) no tienen id real todavía — el
      // servidor lo asigna recién en el primer guardado (ver sessionMigration.js).
      const newBases = action.payload.map((base) => {
        if (base.id != null) return base;
        return { ...base, tempId: base.tempId ?? generateLocalId() };
      });
      state.markers.bases.push(...newBases);
    },
    updatePlanning(state, action) {
      if (action.payload.objetivo) {
        // Migrar planning automáticamente si viene en formato legacy
        const migrated = migratePlanning(action.payload, state.markers);
        state.planning = migrated;
      }
    },
    updatePlanningObjective(state, action) {
      state.planning.objetivo = action.payload;
    },
    updateScene3dOrigin(state, action) {
      state.scene3d.origin = action.payload;
    },
  },
});

export { actions as sessionActions };
export { reducer as sessionReducer };
