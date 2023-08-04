import { createSlice } from '@reduxjs/toolkit';

const { reducer, actions } = createSlice({
  name: 'mission',
  initialState: {
    name: "Mission no loaded",
    home:[0,0],
    route: [],
    attributes: {},
  },
  reducers: {
    updateMission(state, action) {
      state.name = action.payload.name;
      state.route = [];
      state.attributes = {};
      console.log(action.payload.mission)
      if(action.payload.mission.version == "3") {
        console.log("version 3")
        console.log("longitud"+action.payload.mission.route.length)
        for(let n_uav = 0; n_uav < action.payload.mission.route.length; n_uav++){
          console.log("uav - "+n_uav)
          state.route[n_uav] = {}; 
          if(action.payload.mission.route[n_uav].hasOwnProperty('uav')){
            state.route[n_uav]['uav'] = action.payload.mission.route[n_uav].uav;
          }
          state.route[n_uav]['id'] = n_uav;
          state.route[n_uav]['name'] = action.payload.mission.route[n_uav].name;
          state.route[n_uav]['wp'] = [] ;
          state.route[n_uav]['attributes'] = {} ;
          state.home = action.payload.mission.route[n_uav]['wp'][0].pos;
          //waipoints
          for(let wp_n = 0; wp_n < action.payload.mission.route[n_uav]['wp'].length; wp_n++){
            state.route[n_uav]['wp'][wp_n] = {};
            state.route[n_uav]['wp'][wp_n]["pos"] = action.payload.mission.route[n_uav]['wp'][wp_n].pos;
            state.route[n_uav]['wp'][wp_n]["yaw"] = action.payload.mission.route[n_uav]['wp'][wp_n].yaw;
            state.route[n_uav]['wp'][wp_n]["gimbal"] = action.payload.mission.route[n_uav]['wp'][wp_n].gimbal;
            if(action.payload.mission.route[n_uav]['wp'][wp_n].hasOwnProperty('action')){
              state.route[n_uav]['wp'][wp_n]["action"] = action.payload.mission.route[n_uav]['wp'][wp_n].action;
            }
          }
          if(action.payload.mission.hasOwnProperty('attributes')){
            console.log("global attributes")
          if (action.payload.mission.attributes.hasOwnProperty('mode_landing')){
            console.log("global attributes mode landing")
            state.route[n_uav]['attributes']['mode_landing'] = action.payload.mission.attributes["mode_landing"];  
          }
          if (action.payload.mission.attributes.hasOwnProperty('mode_yaw')){
            state.route[n_uav]['attributes']['mode_yaw'] = action.payload.mission.attributes["mode_yaw"];  
          }
          if (action.payload.mission.attributes.hasOwnProperty('mode_gimbal')){
            state.route[n_uav]['attributes']['mode_gimbal'] = action.payload.mission.attributes["mode_gimbal"];  
          }
          if (action.payload.mission.attributes.hasOwnProperty('mode_trace')){
            state.route[n_uav]['attributes']['mode_trace'] = action.payload.mission.attributes["mode_trace"];  
          }
          if (action.payload.mission.attributes.hasOwnProperty('idle_vel')){
            state.route[n_uav]['attributes']['idle_vel'] = action.payload.mission.attributes["idle_vel"];  
          }}
          if (action.payload.mission.route[n_uav].hasOwnProperty('attributes')){
            console.log("have atribute")
            if (action.payload.mission.route[n_uav]['attributes'].hasOwnProperty('mode_landing')){
              console.log("have modelanding"+n_uav)
              state.route[n_uav]['attributes']['mode_landing'] = action.payload.mission.route[n_uav]['attributes']["mode_landing"];  
            }
            if (action.payload.mission.route[n_uav]['attributes'].hasOwnProperty('mode_yaw')){
              state.route[n_uav]['attributes']['mode_yaw'] = action.payload.mission.route[n_uav]['attributes']["mode_yaw"];  
            }
            if (action.payload.mission.route[n_uav]['attributes'].hasOwnProperty('mode_gimbal')){
              state.route[n_uav]['attributes']['mode_gimbal'] = action.payload.mission.route[n_uav]['attributes']["mode_gimbal"];  
            }
            if (action.payload.mission.route[n_uav]['attributes'].hasOwnProperty('mode_trace')){
              state.route[n_uav]['attributes']['mode_trace'] = action.payload.mission.route[n_uav]['attributes']["mode_trace"];  
            }
            if (action.payload.mission.route[n_uav]['attributes'].hasOwnProperty('idle_vel')){
              state.route[n_uav]['attributes']['idle_vel'] = action.payload.mission.route[n_uav]['attributes']["idle_vel"];  
            }
          }else{
            if (action.payload.mission.route[n_uav].hasOwnProperty('mode_landing')){
              console.log("have modelanding"+n_uav)
              state.route[n_uav]['attributes']['mode_landing'] = action.payload.mission.route[n_uav]["mode_landing"];  
            }
            if (action.payload.mission.route[n_uav].hasOwnProperty('mode_yaw')){
              state.route[n_uav]['attributes']['mode_yaw'] = action.payload.mission.route[n_uav]["mode_yaw"];  
            }
            if (action.payload.mission.route[n_uav].hasOwnProperty('mode_gimbal')){
              state.route[n_uav]['attributes']['mode_gimbal'] = action.payload.mission.route[n_uav]["mode_gimbal"];  
            }
            if (action.payload.mission.route[n_uav].hasOwnProperty('mode_trace')){
              state.route[n_uav]['attributes']['mode_trace'] = action.payload.mission.route[n_uav]["mode_trace"];  
            }
            if (action.payload.mission.route[n_uav].hasOwnProperty('idle_vel')){
              state.route[n_uav]['attributes']['idle_vel'] = action.payload.mission.route[n_uav]["idle_vel"];  
            }
          } 
        }
      }else{
        console.log("mission < 3")
        if (action.payload.mission.hasOwnProperty('mode_landing')){
          state.attributes["mode_landing"] = action.payload.mission["mode_landing"];  
        }
        if (action.payload.mission.hasOwnProperty('mode_yaw')){
          state.attributes["mode_yaw"] = action.payload.mission["mode_yaw"];  
        }
        for(let n_uav = 1; n_uav <= action.payload.mission["uav_n"]; n_uav++){
          if (action.payload.mission.hasOwnProperty("uav_"+n_uav)){
            state.route[n_uav] = {}; 
            //state.route[n_uav]['n'] = action.payload.mission["uav_"+n_uav]['wp_n'];
            state.route[n_uav]['id'] = n_uav;
            state.route[n_uav]['uav'] = "uav_"+n_uav;
            state.route[n_uav]['name'] = "uav_"+n_uav;
            state.route[n_uav]['wp'] = [] ;
            state.route[n_uav]['attributes'] = {} ;
            state.home = action.payload.mission["uav_"+n_uav]['wp_0'];
            //waipoints
            for(let wp_n = 0; wp_n < action.payload.mission["uav_"+n_uav]['wp_n']; wp_n++){
              state.route[n_uav]['wp'][wp_n]={};
              if(action.payload.mission["uav_"+n_uav]['wp_'+wp_n].length==3){
                state.route[n_uav]['wp'][wp_n]["yaw"] = 0
                state.route[n_uav]['wp'][wp_n]["pos"] = action.payload.mission["uav_"+n_uav]['wp_'+wp_n];
              }else{
                state.route[n_uav]['wp'][wp_n]["yaw"] = action.payload.mission["uav_"+n_uav]['wp_'+wp_n][3]
                state.route[n_uav]['wp'][wp_n]["pos"] = action.payload.mission["uav_"+n_uav]['wp_'+wp_n].slice(0, -1)
              }
            }
            // Attributes
            if (action.payload.mission.hasOwnProperty('mode_landing')){
              state.route[n_uav]['attributes']['mode_landing'] = action.payload.mission["mode_landing"];  
            }
            if (action.payload.mission.hasOwnProperty('mode_yaw')){
              state.route[n_uav]['attributes']['mode_yaw'] = action.payload.mission["mode_yaw"];  
            }
            if (action.payload.mission.hasOwnProperty('idle_vel')){
              state.route[n_uav]['attributes']['idle_vel'] = action.payload.mission["idle_vel"];  
            }
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
      }
    },
    clearMission(state,action){
      console.log("clear mission")
      state.name = "Mission no loaded";
      state.route = [];
      state.attributes = {};
    },
  },
});

export { actions as missionActions };
export { reducer as missionReducer };
