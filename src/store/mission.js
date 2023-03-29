import { createSlice } from '@reduxjs/toolkit';

const { reducer, actions } = createSlice({
  name: 'mission',
  initialState: {
    name: null,
    route: {},
    attributes: {},
  },
  reducers: {
    updateMission(state, action) {
      state.name = action.payload.name;
      state.route = {};
      state.attributes = {};
      for(let n_uav = 1; n_uav <= action.payload.mission["uav_n"]; n_uav++){
        state.route[n_uav] = action.payload.mission["uav_"+n_uav];
        //for(let n_wp = 0 ; n_wp < action.payload.mission["uav_"+n_uav]["wp_n"]; n_wp++){  }

      }
      if (action.payload.mission.hasOwnProperty('mode_landing')){
        state.attributes["mode_landing"] = action.payload.mission["mode_landing"];  
      }
      if (action.payload.mission.hasOwnProperty('mode_yaw')){
        state.attributes["mode_yaw"] = action.payload.mission["mode_yaw"];  
      }
    },
  },
});

export { actions as missionActions };
export { reducer as missionReducer };