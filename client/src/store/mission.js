import { createSlice } from '@reduxjs/toolkit';
import { RuteConvert, RuteConvertlegacy } from '../Mapview/MissionConvert';

const defaultAttributes = {
  max_vel: 12,
  idle_vel: 3,
  mode_yaw: 2,
  mode_gimbal: 0,
  mode_trace: 0,
  mode_landing: 2,
};

const { reducer, actions } = createSlice({
  name: 'mission',
  initialState: {
    name: 'Mission no loaded',
    description: '',
    home: [0, 0],
    route: [],
    attributes: {},
    selectpoint: { id: -1 },
    groupRouteMode: false,
  },
  reducers: {
    // Existing actions
    updateWpPos(state, action) {
      const { route_id, wp_id, lat, lng } = action.payload;
      if (route_id >= 0 && state.route[route_id]?.wp[wp_id]) {
        state.route[route_id].wp[wp_id].pos[0] = lat;
        state.route[route_id].wp[wp_id].pos[1] = lng;
      }
    },
    selectpoint(state, action) {
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
      state.description = action.payload.description || '';

      if (action.payload.version == '3') {
        state.route = RuteConvert(action.payload.route);
      } else {
        state.route = RuteConvertlegacy(action.payload);
      }
      if (state.route.length > 0) {
        state.home = state.route[0].wp[0].pos;
      }
    },
    clearMission(state) {
      state.name = 'Mission no loaded';
      state.description = '';
      state.route = [];
      state.attributes = {};
      state.selectpoint = { id: -1 };
    },

    // New granular actions
    updateName(state, action) {
      state.name = action.payload;
    },
    updateDescription(state, action) {
      state.description = action.payload;
    },
    setGroupRouteMode(state, action) {
      state.groupRouteMode = action.payload;
    },
    createNewMission(state, action) {
      state.name = action.payload?.name || 'new Mission';
      state.description = action.payload?.description || '';
      state.route = [];
      state.selectpoint = { id: -1 };
    },
    addRoute(state) {
      const newId = state.route.length > 0
        ? state.route[state.route.length - 1].id + 1
        : 0;
      state.route.push({
        name: '',
        uav: '',
        id: newId,
        attributes: { ...defaultAttributes },
        wp: [],
      });
    },
    deleteRoute(state, action) {
      const routeIndex = action.payload;
      if (routeIndex >= 0 && routeIndex < state.route.length) {
        state.route.splice(routeIndex, 1);
      }
    },
    updateRoute(state, action) {
      const { index, field, value } = action.payload;
      if (state.route[index]) {
        state.route[index][field] = value;
      }
    },
    updateRouteAttribute(state, action) {
      const { index, attribute, value } = action.payload;
      if (state.route[index]?.attributes) {
        state.route[index].attributes[attribute] = value;
      }
    },
    addWaypoint(state, action) {
      const { routeIndex, waypoint } = action.payload;
      if (state.route[routeIndex]) {
        const newWp = waypoint || {
          pos: [0, 0, 10],
          action: {},
        };
        state.route[routeIndex].wp.push(newWp);
      }
    },
    deleteWaypoint(state, action) {
      const { routeIndex, wpIndex } = action.payload;
      if (state.route[routeIndex]?.wp[wpIndex]) {
        state.route[routeIndex].wp.splice(wpIndex, 1);
      }
    },
    updateWaypoint(state, action) {
      const { routeIndex, wpIndex, field, value } = action.payload;
      if (state.route[routeIndex]?.wp[wpIndex]) {
        state.route[routeIndex].wp[wpIndex][field] = value;
      }
    },
    updateWaypointPos(state, action) {
      const { routeIndex, wpIndex, pos } = action.payload;
      if (state.route[routeIndex]?.wp[wpIndex]) {
        state.route[routeIndex].wp[wpIndex].pos = pos;
      }
    },
    // Move waypoint with optional group mode (moves entire route)
    moveWaypoint(state, action) {
      const { route_id, wp_id, lat, lng, groupMode } = action.payload;
      if (route_id < 0 || !state.route[route_id]?.wp[wp_id]) return;

      if (groupMode) {
        // Move entire route by the same delta
        const currentWp = state.route[route_id].wp[wp_id];
        const dif_lat = lat - currentWp.pos[0];
        const dif_lng = lng - currentWp.pos[1];
        state.route[route_id].wp.forEach(wp => {
          wp.pos[0] += dif_lat;
          wp.pos[1] += dif_lng;
        });
      } else {
        // Move only the selected waypoint
        state.route[route_id].wp[wp_id].pos[0] = lat;
        state.route[route_id].wp[wp_id].pos[1] = lng;
      }
    },
    // Update single position index (lat/lng/alt)
    updateWaypointPosIndex(state, action) {
      const { routeIndex, wpIndex, posIndex, value } = action.payload;
      if (state.route[routeIndex]?.wp[wpIndex]?.pos) {
        state.route[routeIndex].wp[wpIndex].pos[posIndex] = value;
      }
    },
    // Update waypoint action value
    updateWaypointAction(state, action) {
      const { routeIndex, wpIndex, actionKey, value } = action.payload;
      if (state.route[routeIndex]?.wp[wpIndex]) {
        if (!state.route[routeIndex].wp[wpIndex].action) {
          state.route[routeIndex].wp[wpIndex].action = {};
        }
        state.route[routeIndex].wp[wpIndex].action[actionKey] = value;
      }
    },
    // Add new action to waypoint
    addWaypointAction(state, action) {
      const { routeIndex, wpIndex, actionKey, value } = action.payload;
      if (state.route[routeIndex]?.wp[wpIndex]) {
        if (!state.route[routeIndex].wp[wpIndex].action) {
          state.route[routeIndex].wp[wpIndex].action = {};
        }
        state.route[routeIndex].wp[wpIndex].action[actionKey] = value;
      }
    },
    // Remove action from waypoint
    removeWaypointAction(state, action) {
      const { routeIndex, wpIndex, actionKey } = action.payload;
      if (state.route[routeIndex]?.wp[wpIndex]?.action) {
        delete state.route[routeIndex].wp[wpIndex].action[actionKey];
      }
    },
    // Copy waypoint (duplicate after current position)
    copyWaypoint(state, action) {
      const { routeIndex, wpIndex } = action.payload;
      if (state.route[routeIndex]?.wp[wpIndex]) {
        const wpCopy = JSON.parse(JSON.stringify(state.route[routeIndex].wp[wpIndex]));
        state.route[routeIndex].wp.splice(wpIndex + 1, 0, wpCopy);
      }
    },
    // Move waypoint order (up/down in list)
    moveWaypointOrder(state, action) {
      const { routeIndex, wpIndex, direction } = action.payload;
      const wpArray = state.route[routeIndex]?.wp;
      if (!wpArray || wpIndex < 0 || wpIndex >= wpArray.length) return;

      const newIndex = wpIndex + direction;
      if (newIndex < 0) {
        // Move to end
        const [wp] = wpArray.splice(wpIndex, 1);
        wpArray.push(wp);
      } else if (newIndex >= wpArray.length) {
        // Move to beginning
        const [wp] = wpArray.splice(wpIndex, 1);
        wpArray.unshift(wp);
      } else {
        // Swap positions
        const [wp] = wpArray.splice(wpIndex, 1);
        wpArray.splice(newIndex, 0, wp);
      }
    },
  },
});

export { actions as missionActions };
export { reducer as missionReducer };
