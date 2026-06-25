import { createSlice } from '@reduxjs/toolkit';

// status values mirror server MISSION_STATUS / ROUTE_STATUS constants
// Mission: 'init' | 'running' | 'completed'
// Route:   'init' | 'commanded' | 'running' | 'completed'

const { reducer: activeMissionsReducer, actions: activeMissionsActions } = createSlice({
  name: 'activeMissions',
  initialState: {
    // { [missionId]: { id, name, status, uav, initTime, routes: { [deviceId]: { deviceId, status, currentWp, totalWp } } } }
    items: {},
    selectedMissionId: null,
  },
  reducers: {
    setMissions(state, action) {
      // Full replace from initial REST fetch — payload: Mission[]
      state.items = {};
      for (const m of action.payload) {
        state.items[m.id] = {
          id: m.id,
          name: m.name,
          status: m.status,
          uav: m.uav ?? [],
          initTime: m.initTime,
          endTime: m.endTime ?? null,
          routes: {},
        };
      }
    },

    setRoutes(state, action) {
      // Merge routes from GET /api/missions/routes — payload: MissionRoute[]
      for (const r of action.payload) {
        if (!state.items[r.missionId]) continue;
        state.items[r.missionId].routes[r.deviceId] = {
          deviceId: r.deviceId,
          status: r.status,
          currentWp: r.currentWp,
          totalWp: r.totalWp,
        };
      }
    },

    upsertMission(state, action) {
      // Insert or update a single mission — payload: Mission DB record
      const m = action.payload;
      const existing = state.items[m.id];
      state.items[m.id] = {
        id: m.id,
        name: m.name,
        status: m.status,
        uav: m.uav ?? [],
        initTime: m.initTime,
        endTime: m.endTime ?? null,
        routes: existing?.routes ?? {},
      };
    },

    updateProgress(state, action) {
      // From WS missionProgress: { missionId, deviceId, currentWp, totalWp, completed, anomalies, wpEstimate, confidence }
      const { missionId, deviceId, currentWp, totalWp, anomalies = [], wpEstimate = null, confidence = null } = action.payload;
      const mission = state.items[missionId];
      // If mission is unknown, create a placeholder — SocketController will fetch full data
      if (!mission) {
        state.items[missionId] = {
          id: missionId,
          name: `Mission ${missionId}`,
          status: 'running',
          uav: [],
          initTime: null,
          endTime: null,
          routes: {
            [deviceId]: { deviceId, status: 'running', currentWp, totalWp, anomalies, wpEstimate, confidence },
          },
        };
        return;
      }
      if (!mission.routes[deviceId]) {
        mission.routes[deviceId] = { deviceId, status: 'running', currentWp, totalWp, anomalies, wpEstimate, confidence };
      } else {
        mission.routes[deviceId].currentWp = currentWp;
        mission.routes[deviceId].totalWp = totalWp;
        mission.routes[deviceId].status = action.payload.completed ? 'completed' : 'running';
        mission.routes[deviceId].anomalies = anomalies;
        mission.routes[deviceId].wpEstimate = wpEstimate;
        mission.routes[deviceId].confidence = confidence;
      }
      mission.status = 'running';
    },

    completeMission(state, action) {
      // From WS missionCompleted: { missionId }
      const mission = state.items[action.payload.missionId];
      if (mission) mission.status = 'completed';
    },

    selectMission(state, action) {
      state.selectedMissionId = action.payload;
    },
  },
});

export { activeMissionsReducer, activeMissionsActions };
