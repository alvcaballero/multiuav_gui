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
      // From WS missionProgress: { missionId, deviceId, currentWp, totalWp, completed,
      // anomalies, wpEstimate, confidence, missionStatus?, routeStatus? }.
      // missionStatus/routeStatus are optional and carry the real server status (e.g.
      // 'init'/'loaded' right after a manual load) — default to 'running' for the
      // in-flight progress updates, which don't send them.
      const {
        missionId,
        deviceId,
        currentWp,
        totalWp,
        anomalies = [],
        wpEstimate = null,
        confidence = null,
        missionStatus,
        routeStatus,
      } = action.payload;
      const mission = state.items[missionId];
      // If mission is unknown, create a placeholder — SocketController will fetch full data
      if (!mission) {
        state.items[missionId] = {
          id: missionId,
          name: `Mission ${missionId}`,
          status: missionStatus ?? 'running',
          uav: [],
          initTime: null,
          endTime: null,
          routes: {
            [deviceId]: { deviceId, status: routeStatus ?? 'running', currentWp, totalWp, anomalies, wpEstimate, confidence },
          },
        };
        return;
      }
      if (!mission.routes[deviceId]) {
        mission.routes[deviceId] = {
          deviceId,
          status: routeStatus ?? 'running',
          currentWp,
          totalWp,
          anomalies,
          wpEstimate,
          confidence,
        };
      } else {
        mission.routes[deviceId].currentWp = currentWp;
        mission.routes[deviceId].totalWp = totalWp;
        mission.routes[deviceId].status = routeStatus ?? (action.payload.completed ? 'completed' : 'running');
        mission.routes[deviceId].anomalies = anomalies;
        mission.routes[deviceId].wpEstimate = wpEstimate;
        mission.routes[deviceId].confidence = confidence;
      }
      mission.status = missionStatus ?? 'running';
    },

    completeMission(state, action) {
      // From WS missionCompleted: { missionId, status? }. status carries the real
      // server value ('finish' | 'finish_errors'); default to 'completed' if absent.
      const mission = state.items[action.payload.missionId];
      if (mission) mission.status = action.payload.status ?? 'completed';
    },

    selectMission(state, action) {
      state.selectedMissionId = action.payload;
    },
  },
  extraReducers: (builder) => {
    // Clearing or starting a new mission in the editor means the user is starting
    // fresh — drop the active selection so commandMission no longer targets it.
    // (Cross-slice: these actions live in the mission slice.)
    //
    // NOTE: we deliberately do NOT react to 'mission/updateMission' here.
    // updateMission fires both when loading a *new* mission (file/planner — where
    // deselecting is correct) AND from MissionTrackingPanel.handleSelect when the
    // user selects an active mission and we load its plan onto the map (where
    // deselecting would immediately undo the selection). Loaders that should drop
    // the selection dispatch selectMission(null) explicitly instead.
    builder
      .addCase('mission/clearMission', (state) => {
        state.selectedMissionId = null;
      })
      .addCase('mission/createNewMission', (state) => {
        state.selectedMissionId = null;
      });
  },
});

export { activeMissionsReducer, activeMissionsActions };

// A mission can be commanded only when it has routes and at least one is LOADED
// (loaded but not yet commanded). Routes already commanded/running are skipped.
const isCommandable = (mission) => {
  const routes = Object.values(mission?.routes ?? {});
  return routes.length > 0 && routes.some((r) => r.status === 'loaded');
};

/**
 * Resolves which missionId the user can command right now: the selected mission
 * (activeMissions.selectedMissionId), but only if it still has LOADED routes.
 * A successful manual load selects the created mission; after a page refresh the
 * user picks it again from the tracking panel. Returns null otherwise.
 */
export const getCommandableMissionId = (state) => {
  const { items, selectedMissionId } = state.activeMissions;
  if (selectedMissionId != null && isCommandable(items[selectedMissionId])) return selectedMissionId;

  return null;
};
