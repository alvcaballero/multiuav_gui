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
          for(let wp_n = 0; wp_n < action.payload.mission["uav_"+n_uav]['wp_n']; wp_n++){
            state.route[n_uav]['wp'][wp_n] = action.payload.mission["uav_"+n_uav]['wp_'+wp_n];
          }
        }

      }
    },
    clearMission(state,action){
      state.name = "Mission no loaded";
      state.route = {};
      state.home = {};
      state.attributes = {};
    },
  },
});

export { actions as missionActions };
export { reducer as missionReducer };
