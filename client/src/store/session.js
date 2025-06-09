import { createSlice } from '@reduxjs/toolkit';

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
    markers: {},
    planning: {},
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
      console.log('Updating positions', action.payload);

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
        state.markers = action.payload;
      }
    },
    addMarkerElement(state, action) {
      state.markers.elements.push(...action.payload);
    },
    addMarkerBase(state, action) {
      state.markers.bases.push(...action.payload);
    },
    updatePlanning(state, action) {
      if (action.payload.objetivo) {
        state.planning = action.payload;
      }
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
