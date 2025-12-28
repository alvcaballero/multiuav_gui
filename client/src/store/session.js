import { createSlice } from '@reduxjs/toolkit';
import { migrateMarkers, migratePlanning, generateBaseId } from './sessionMigration';

const { reducer, actions } = createSlice({
  name: 'session',
  initialState: {
    server: null,
    user: {},
    socket: null,
    includeLogs: false,
    logs: [],
    positions: {},
    history: {},
    camera: {},
    markers: {
      bases: [],
      elements: [],
    },
    planning: {
      id: null,
      objetivo: {},
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
      state.server.rosState = action.payload;
    },
    updateUser(state, action) {
      state.user = action.payload;
    },
    updateSocket(state, action) {
      state.socket = action.payload;
    },
    updatePositions(state, action) {
      //console.log('Updating positions', action.payload);
      const liveRoutes = state.user?.attributes?.mapLiveRoutes || state.server.attributes.mapLiveRoutes || 'none';
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
      // Asegurar que las nuevas bases tengan ID
      const newBases = action.payload.map((base) => {
        if (base.id) return base;
        return { ...base, id: generateBaseId() };
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
    // Nuevas acciones para trabajar con la estructura mejorada
    setBaseAssignment(state, action) {
      const { baseId, device, settings } = action.payload;
      const existingIndex = state.planning.assignments.findIndex((a) => a.baseId === baseId);

      if (existingIndex >= 0) {
        // Actualizar asignación existente
        state.planning.assignments[existingIndex] = {
          baseId,
          device,
          settings: settings || state.planning.assignments[existingIndex].settings,
        };
      } else {
        // Crear nueva asignación
        state.planning.assignments.push({
          baseId,
          device,
          settings: settings || state.planning.defaultSettings,
        });
      }
    },
    removeBaseAssignment(state, action) {
      const baseId = action.payload;
      state.planning.assignments = state.planning.assignments.filter((a) => a.baseId !== baseId);
    },
    updateBaseSettings(state, action) {
      const { baseId, settings } = action.payload;
      const assignment = state.planning.assignments.find((a) => a.baseId === baseId);
      if (assignment) {
        assignment.settings = { ...assignment.settings, ...settings };
      }
    },
    updateDefaultSettings(state, action) {
      state.planning.defaultSettings = { ...state.planning.defaultSettings, ...action.payload };
    },
    updatePlanningObjective(state, action) {
      state.planning.objetivo = action.payload;
    },
    updatePlanningLocations(state, action) {
      state.planning.loc = action.payload;
    },
    updateSettingsSchema(state, action) {
      state.planning.settingsSchema = action.payload;
    },
    updateCamera(state, action) {
      if (Object.keys(action.payload).length > 0) {
        action.payload.forEach((camera) => {
          state.camera[camera.deviceId] = camera;
        });
      }
    },
    updateScene3dOrigin(state, action) {
      state.scene3d.origin = action.payload;
    },
  },
});

export { actions as sessionActions };
export { reducer as sessionReducer };
