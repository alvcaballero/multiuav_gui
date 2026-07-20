import { createSlice } from '@reduxjs/toolkit';

// status values mirror server MISSION_STATUS / ROUTE_STATUS constants exactly
// (see server/config/status.js) — always the raw server string, never derived here.
// Mission: init | planning | running | finish | finish_errors | done | cancelled | error
// Route:   init | loaded | commanded | running | complete | end | cancelled | error

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

    upsertRoute(state, action) {
      // From WS routeUpdated: the MissionRoute DB row as-is — { missionId, deviceId,
      // status, currentWp, totalWp, anomalies?, wpEstimate?, confidence?, ... }.
      // status is always the real server ROUTE_STATUS value; no derivation needed here.
      const {
        missionId,
        deviceId,
        currentWp,
        totalWp,
        status,
        anomalies = [],
        wpEstimate = null,
        confidence = null,
      } = action.payload;
      const mission = state.items[missionId];
      // If mission is unknown, create a placeholder — SocketController will fetch full data.
      // Its own status isn't known yet (that arrives via a separate missionUpdated event);
      // MISSION_UPDATED is emitted before ROUTE_UPDATED for a brand new mission, so this
      // is only a fallback for a route arriving out of order.
      if (!mission) {
        state.items[missionId] = {
          id: missionId,
          name: `Mission ${missionId}`,
          status: 'running',
          uav: [],
          initTime: null,
          endTime: null,
          routes: {
            [deviceId]: { deviceId, status, currentWp, totalWp, anomalies, wpEstimate, confidence },
          },
        };
        return;
      }
      mission.routes[deviceId] = {
        deviceId,
        status,
        currentWp,
        totalWp,
        anomalies,
        wpEstimate,
        confidence,
      };
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
  if (selectedMissionId != null && isCommandable(items[selectedMissionId]))
    return selectedMissionId;

  return null;
};
