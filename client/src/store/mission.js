import { createSlice, createAsyncThunk } from '@reduxjs/toolkit';
import { RuteConvert, RuteConvertlegacy } from '../map/MissionConvert';

const defaultAttributes = {
  max_vel: 12,
  idle_vel: 2,
  mode_yaw: 3,
  mode_gimbal: 1,
  mode_trace: 0,
  mode_landing: 2,
};

export const applyUavTypeDefaults = createAsyncThunk(
  'mission/applyUavTypeDefaults',
  async ({ routeIndex, uavType }) => {
    const response = await fetch(`/api/category/attributesdefaults/${uavType}`);
    if (!response.ok) throw new Error('Failed to fetch attribute defaults');
    const defaults = await response.json();
    return { routeIndex, defaults };
  }
);

const { reducer: missionReducerBase, actions } = createSlice({
  name: 'mission',
  initialState: {
    name: 'Mission no loaded',
    description: '',
    home: [0, 0],
    route: [],
    attributes: {},
    selectpoint: { id: -1 },
    groupRouteMode: false,
    // Elevation profile cache
    elevation: {
      profile: [], // Array of {name, data, color} for each route
      location: [], // Cached waypoint locations [lat, lon, alt]
      selectRT: -1, // Selected route filter (-1 = all)
      loading: false, // Loading indicator
    },
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
      // Clear elevation cache
      state.elevation.profile = [];
      state.elevation.location = [];
      state.elevation.selectRT = -1;
      state.elevation.loading = false;
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
      const newId = state.route.length > 0 ? state.route[state.route.length - 1].id + 1 : 0;
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
      const { routeIndex, waypoint, insertAt } = action.payload;
      if (state.route[routeIndex]) {
        const newWp = waypoint || {
          pos: [0, 0, 10],
          action: {},
        };
        if (insertAt !== undefined && insertAt >= 0 && insertAt <= state.route[routeIndex].wp.length) {
          state.route[routeIndex].wp.splice(insertAt, 0, newWp);
        } else {
          state.route[routeIndex].wp.push(newWp);
        }
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
        state.route[route_id].wp.forEach((wp) => {
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

    // Elevation profile actions
    setElevationProfile(state, action) {
      state.elevation.profile = action.payload;
    },
    setElevationLocation(state, action) {
      state.elevation.location = action.payload;
    },
    setElevationSelectRT(state, action) {
      state.elevation.selectRT = action.payload;
    },
    setElevationLoading(state, action) {
      state.elevation.loading = action.payload;
    },
    updateElevationProfile(state, action) {
      // For updating specific route elevation data (e.g., when altitude changes)
      const { routeIndex, data } = action.payload;
      if (state.elevation.profile[routeIndex]) {
        state.elevation.profile[routeIndex].data = data;
      }
    },
    clearElevation(state) {
      state.elevation.profile = [];
      state.elevation.location = [];
      state.elevation.selectRT = -1;
      state.elevation.loading = false;
    },

    // Rotate all waypoints around a center point (lat/lng in degrees)
    // Uses local-metric projection: 1° lat ≈ 111320 m, 1° lng ≈ 111320 * cos(lat) m
    rotateMission(state, action) {
      const { angleDeg, routeIndex } = action.payload; // routeIndex = -1 → all routes
      const angleRad = (angleDeg * Math.PI) / 180;

      // Collect all waypoints to compute centroid
      const routes = routeIndex >= 0 ? [state.route[routeIndex]] : state.route;

      let sumLat = 0,
        sumLng = 0,
        count = 0;
      routes.forEach((route) => {
        route.wp.forEach((wp) => {
          sumLat += wp.pos[0];
          sumLng += wp.pos[1];
          count++;
        });
      });
      if (count === 0) return;

      const cLat = sumLat / count;
      const cLng = sumLng / count;
      const cosLat = Math.cos((cLat * Math.PI) / 180);
      const mPerDegLat = 111320;
      const mPerDegLng = 111320 * cosLat;

      const cosA = Math.cos(angleRad);
      const sinA = Math.sin(angleRad);

      // Yaw is NED/aeronautic convention: 0=North, clockwise positive.
      // Position rotates CCW for positive angleDeg (math convention).
      // To keep both consistent, negate angleDeg when rotating yaw.
      // Normalize result to [-180, 180).
      const rotateYaw = (yawDeg) => {
        const r = ((yawDeg - angleDeg) % 360 + 360) % 360;
        return r >= 180 ? r - 360 : r;
      };

      routes.forEach((route) => {
        route.wp.forEach((wp) => {
          // Project to local meters relative to centroid
          const dx = (wp.pos[1] - cLng) * mPerDegLng;
          const dy = (wp.pos[0] - cLat) * mPerDegLat;
          // Rotate position
          const rx = dx * cosA - dy * sinA;
          const ry = dx * sinA + dy * cosA;
          // Back to lat/lng
          wp.pos[0] = cLat + ry / mPerDegLat;
          wp.pos[1] = cLng + rx / mPerDegLng;
          // Rotate yaw: direct field
          if (wp.yaw != null) wp.yaw = rotateYaw(wp.yaw);
          // Rotate yaw: inside action object
          if (wp.action?.yaw != null) wp.action.yaw = rotateYaw(wp.action.yaw);
        });
      });
    },

    // Translate all waypoints by delta lat/lng
    translateMission(state, action) {
      const { deltaLat, deltaLng, routeIndex } = action.payload;
      const routes = routeIndex >= 0 ? [state.route[routeIndex]] : state.route;

      routes.forEach((route) => {
        route.wp.forEach((wp) => {
          wp.pos[0] += deltaLat;
          wp.pos[1] += deltaLng;
        });
      });
    },
    removeElevationRoute(state, action) {
      // Remove specific routes from elevation cache by indices
      const indicesToKeep = action.payload;
      state.elevation.profile = indicesToKeep.map((i) => state.elevation.profile[i]).filter(Boolean);
      state.elevation.location = indicesToKeep.map((i) => state.elevation.location[i]).filter(Boolean);
    },
  },
  extraReducers: (builder) => {
    builder.addCase(applyUavTypeDefaults.fulfilled, (state, action) => {
      const { routeIndex, defaults } = action.payload;
      if (state.route[routeIndex]) {
        state.route[routeIndex].attributes = { ...state.route[routeIndex].attributes, ...defaults };
      }
    });
  },
});

export { actions as missionActions };
export { missionReducerBase as missionReducer };
