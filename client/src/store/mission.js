import { createSlice } from '@reduxjs/toolkit';
import { RuteConvert, RuteConvertlegacy } from '../Mapview/MissionConvert';

const { reducer, actions } = createSlice({
  name: 'mission',
  initialState: {
    name: 'Mission no loaded',
    description: '',
    home: [0, 0],
    route: [],
    attributes: {},
    selectpoint: { id: -1 },
  },
  reducers: {
    updateWpPos(state, action) {
      if (action.payload.route_id >= 0) {
        state.route[action.payload.route_id]['wp'][action.payload.wp_id]['pos'][0] =
          action.payload.lat;
        state.route[action.payload.route_id]['wp'][action.payload.wp_id]['pos'][1] =
          action.payload.lng;
      }
    },
    selectpoint(state, action) {
      console.log('selec point');
      state.selectpoint = action.payload;
    },
    reloadMission(state, action) {
      state.route = action.payload;
    },
    reloadName(state, action) {
      state.name = action.payload.name;
      state.description = action.payload.description;
    },
    updateMission(state, action) {
      state.name = action.payload.name;
      //state.attributes = {};

      if (action.payload.version == '3') {
        let n_uav = 0;
        state.home = action.payload.route[n_uav]['wp'][0].pos;
        state.route = RuteConvert(action.payload.route);
      } else {
        let n_uav = 1;
        state.home = action.payload['uav_' + n_uav]['wp_0'];
        state.route = RuteConvertlegacy(action.payload);
      }
    },
    clearMission(state, action) {
      console.log('clear mission');
      state.name = 'Mission no loaded';
      state.route = [];
      state.attributes = {};
    },
  },
});

export { actions as missionActions };
export { reducer as missionReducer };
