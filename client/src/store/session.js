import { createSlice } from '@reduxjs/toolkit';

const { reducer, actions } = createSlice({
  name: 'session',
  initialState: {
    server: null,
    user: null,
    socket: null,
    positions: {},
    history: {},
    markers: { bases: [], elements: [] },
    planning: {
      id: 1234,
      name: 'no mission',
      objetivo: 0,
      loc: [],
      meteo: [],
      bases: [],
      settings: {},
    },
  },
  reducers: {
    updateServer(state, action) {
      state.server = action.payload;
    },
    updateUser(state, action) {
      state.user = action.payload;
    },
    updateSocket(state, action) {
      state.socket = action.payload;
    },
    updatePositions(state, action) {
      const liveRoutes =
        state.user.attributes.mapLiveRoutes || state.server.attributes.mapLiveRoutes || 'none';
      const liveRoutesLimit =
        state.user.attributes['web.liveRouteLength'] ||
        state.user.attributes['web.liveRouteLength'] ||
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
      state.markers = action.payload;
    },
    addMarkerElement(state, action) {
      state.markers.elements.push(...action.payload);
    },
    addMarkerBase(state, action) {
      state.markers.bases.push(...action.payload);
    },
    updatePlanning(state, action) {
      state.planning = action.payload;
    },
  },
});

export { actions as sessionActions };
export { reducer as sessionReducer };
