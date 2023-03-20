import { createSlice } from '@reduxjs/toolkit';

const { reducer, actions } = createSlice({
  name: 'data',
  initialState: {
    server: null,
    user: null,
    socket: null,
    camera: {},
    positions: {},
    history: {},
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
      const liveRoutes = state.user.attributes.mapLiveRoutes || state.server.attributes.mapLiveRoutes || 'none';
      const liveRoutesLimit = state.user.attributes['web.liveRouteLength'] || state.user.attributes['web.liveRouteLength'] || 10;
      action.payload.forEach((position) => {
        state.positions[position.deviceId] = position;
        if (liveRoutes !== 'none') {
          const route = state.history[position.deviceId] || [];
          const last = route.at(-1);
          if (!last || (last[0] !== position.longitude && last[1] !== position.latitude)) {
            state.history[position.deviceId] = [...route.slice(1 - liveRoutesLimit), [position.longitude, position.latitude]];
          }
        } else {
          state.history = {};
        }
      });
    },
    updatePosition(state, action) {
      //state.positions[action.payload.deviceId] = action.payload;
      if (state.positions[action.payload.deviceId] === undefined) {
        state.positions[action.payload.deviceId] = {}
      };
      state.positions[action.payload.deviceId]["id"] = action.payload.id;
      state.positions[action.payload.deviceId]["deviceId"] = action.payload.deviceId;
      state.positions[action.payload.deviceId]["latitude"] = action.payload.latitude;
      state.positions[action.payload.deviceId]["longitude"] = action.payload.longitude;
      state.positions[action.payload.deviceId]["altitude"] = action.payload.altitude;
      state.positions[action.payload.deviceId]["deviceTime"] = action.payload.deviceTime;



        

    },
    updateCourse(state, action){
      if (state.positions[action.payload.deviceId] === undefined) {
        state.positions[action.payload.deviceId] = {}
      };
      state.positions[action.payload.deviceId]["course"] = action.payload.course;        
      state.positions[action.payload.deviceId]["attributes"] = {};  
    },
  },
});

export { actions as dataActions };
export { reducer as dataReducer };
