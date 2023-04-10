const state = {
  camera: {},
  positions: {},
  history: {},
};

function updatePosition(state, payload) {
    //state.positions[payload.deviceId] = payload;
    if (state.positions[payload.deviceId] === undefined) {
      state.positions[payload.deviceId] = {attributes:{}}
    };
    if (payload.hasOwnProperty('latitude')){
      state.positions[payload.deviceId]["id"] = payload.id;
      state.positions[payload.deviceId]["deviceId"] = payload.deviceId;
      state.positions[payload.deviceId]["latitude"] = payload.latitude;
      state.positions[payload.deviceId]["longitude"] = payload.longitude;
      state.positions[payload.deviceId]["altitude"] = payload.altitude;
      state.positions[payload.deviceId]["deviceTime"] = payload.deviceTime;
    }
    if (payload.hasOwnProperty('course')){
      state.positions[payload.deviceId]["course"] = payload.course;  
    }
    if (payload.hasOwnProperty('speed')){
      state.positions[payload.deviceId]["speed"] = payload.speed;  
    }
    if (payload.hasOwnProperty('batteryLevel')){
      state.positions[payload.deviceId]['attributes']['batteryLevel'] = payload.batteryLevel;  
    }
}

function updateCamera(state, action) {
      state.camera[payload.deviceId] = payload;
    }