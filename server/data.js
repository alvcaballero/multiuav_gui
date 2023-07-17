 var state = {
  camera: {},
  positions: {},
  history: {},
  devices: {},
};

function updatedevice(payload) {
  state.devices[payload.id] = payload;
}
function updatedeviceIP(payload){
  state.devices[payload.id]["ip"] = payload.ip;
}
function get_device_ns(uav_id){
  return state.devices[uav_id].name
} 
function get_device_category(uav_id){
  return state.devices[uav_id].category
} 


function removedevice(payload) {
  delete state.devices[payload.id];
  console.log(state.devices)
}

 function updatePosition(payload) {
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
      state.positions[payload.deviceId]["deviceTime"] = payload.deviceTime;
    }
    if (payload.hasOwnProperty('altitude')){
      state.positions[payload.deviceId]["altitude"] = payload.altitude;
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
      }else{
        state.positions[payload.deviceId]['attributes']['alarm'] = undefined;
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
  updatedeviceIP,
  get_device_ns,
  get_device_category,
  updatePosition,
  updateCamera,
  removedevice,
};