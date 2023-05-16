 var state = {
  camera: {},
  positions: {},
  history: {},
  devices: {},
};
 function updatedevice( payload) {
  state.devices[payload.id] = payload;
}

 function updatePosition( payload) {
    //state.positions[payload.deviceId] = payload;
    let currentTime = new Date()
    state.devices[payload.deviceId]["lastUpdate"] =currentTime

    if (state.positions[payload.deviceId] === undefined) {
      state.positions[payload.deviceId] = {deviceId:payload.deviceId,attributes:{}}
    };

    if (payload.hasOwnProperty('latitude')){
      //state.positions[payload.deviceId]["deviceId"] = payload.deviceId;
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
    if (payload.hasOwnProperty('uav_state')){
      state.positions[payload.deviceId]['attributes']['protocol'] = payload.protocol; 
      state.positions[payload.deviceId]['attributes']['mission_state'] = payload.mission_state;   
      state.positions[payload.deviceId]['attributes']['wp_reached'] = payload.wp_reached; 
      state.positions[payload.deviceId]['attributes']['uav_state'] = payload.uav_state; 
      state.positions[payload.deviceId]['attributes']['landed_state'] = convert_landed_state(payload.protocol,payload.landed_state); 
    }
    if (payload.hasOwnProperty('threat')){
      //state.positions[payload.deviceId]['attributes']['threat'] = payload.threat; 
      if (payload.threat == true){
        state.positions[payload.deviceId]['attributes']['alarm'] = "threat"; 
      }
    }
}

 function updateCamera( payload) {
      state.camera[payload.deviceId] = payload;
}
function convert_landed_state(protocol,landed_state){
  state_px4_stol=["UNDEFINED","ON GROUND","IN AIR","TAKEOFF","LANDING"]
  state_dji=["STOPED","ON GROUND","IN AIR"]
  if(protocol =='dji' ){
    return state_dji[landed_state]
  }
  if(protocol =='catec' ){
    return landed_state
  }
  return state_px4_stol[landed_state]
}
module.exports = {
  state,
  updatedevice,
  updatePosition,
  updateCamera,
};