import { createSlice } from '@reduxjs/toolkit';

const { reducer, actions } = createSlice({
  name: 'mission',
  initialState: {
    name: "Mission no loaded",
    home:[0,0],
    route: {},
    attributes: {},
  },
  reducers: {
    updateMission(state, action) {
      state.name = action.payload.name;
      state.route = {};
      state.attributes = {};
      console.log(action.payload.mission)
      if (action.payload.mission.hasOwnProperty('mode_landing')){
        state.attributes["mode_landing"] = action.payload.mission["mode_landing"];  
      }
      if (action.payload.mission.hasOwnProperty('mode_yaw')){
        state.attributes["mode_yaw"] = action.payload.mission["mode_yaw"];  
      }
      for(let n_uav = 1; n_uav <= action.payload.mission["uav_n"]; n_uav++){
        if (action.payload.mission.hasOwnProperty("uav_"+n_uav)){
          state.route[n_uav] = {}; 
          state.route[n_uav]['n'] = action.payload.mission["uav_"+n_uav]['wp_n'];
          state.route[n_uav]['id'] = n_uav;
          state.route[n_uav]['name'] = "uav_"+n_uav;
          state.route[n_uav]['wp'] = {} ;
          state.route[n_uav]['attributes'] = {} ;
          state.home = action.payload.mission["uav_"+n_uav]['wp_0'];
          //globa Attributes
          if (action.payload.mission.hasOwnProperty('mode_landing')){
            state.route[n_uav]['attributes']['mode_landing'] = action.payload.mission["mode_landing"];  
          }
          if (action.payload.mission.hasOwnProperty('mode_yaw')){
            state.route[n_uav]['attributes']['mode_yaw'] = action.payload.mission["mode_yaw"];  
          }
          if (action.payload.mission.hasOwnProperty('idle_vel')){
            state.route[n_uav]['attributes']['idle_vel'] = action.payload.mission["idle_vel"];  
          }
          //waipoints
          for(let wp_n = 0; wp_n < action.payload.mission["uav_"+n_uav]['wp_n']; wp_n++){
            state.route[n_uav]['wp'][wp_n] = action.payload.mission["uav_"+n_uav]['wp_'+wp_n];
            if(state.route[n_uav]['wp'][wp_n].length==3){

              state.route[n_uav]['wp'][wp_n].push(0);
            }
          }
          //
          if (action.payload.mission["uav_"+n_uav].hasOwnProperty('attributes')){
            console.log("have atribute")
            if (action.payload.mission["uav_"+n_uav]['attributes'].hasOwnProperty('mode_landing')){
              console.log("have modelanding"+n_uav)
              state.route[n_uav]['attributes']['mode_landing'] = action.payload.mission["uav_"+n_uav]['attributes']["mode_landing"];  
            }
            if (action.payload.mission["uav_"+n_uav]['attributes'].hasOwnProperty('mode_yaw')){
              state.route[n_uav]['attributes']['mode_yaw'] = action.payload.mission["uav_"+n_uav]['attributes']["mode_yaw"];  
            }
            if (action.payload.mission["uav_"+n_uav]['attributes'].hasOwnProperty('idle_vel')){
              state.route[n_uav]['attributes']['idle_vel'] = action.payload.mission["uav_"+n_uav]['attributes']["idle_vel"];  
            }
          }else{
            if (action.payload.mission["uav_"+n_uav].hasOwnProperty('mode_landing')){
              console.log("have modelanding"+n_uav)
              state.route[n_uav]['attributes']['mode_landing'] = action.payload.mission["uav_"+n_uav]["mode_landing"];  
            }
            if (action.payload.mission["uav_"+n_uav].hasOwnProperty('mode_yaw')){
              state.route[n_uav]['attributes']['mode_yaw'] = action.payload.mission["uav_"+n_uav]["mode_yaw"];  
            }
            if (action.payload.mission["uav_"+n_uav].hasOwnProperty('idle_vel')){
              state.route[n_uav]['attributes']['idle_vel'] = action.payload.mission["uav_"+n_uav]["idle_vel"];  
            }
          }
        }
      }
    },
    clearMission(state,action){
      state.name = "Mission no loaded";
      state.home = [0,0];
      state.route = {};
      state.attributes = {};
    },
  },
});

export { actions as missionActions };
export { reducer as missionReducer };
