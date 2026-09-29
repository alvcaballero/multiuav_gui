import { createSlice } from '@reduxjs/toolkit';

// status values mirror server MISSION_STATUS / TASK_STATUS constants exactly
// (see server/config/status.js) — always the raw server string, never derived here.
// Mission: init | planning | running | finish | finish_errors | done | cancelled | error
// Task:    init | loaded | commanded | running | complete | end | cancelled | error | skipped

// Only the fields the tracking UI uses. A device can run several tasks of one
// mission, so tasks are keyed by task id, never by deviceId.
const taskFields = (t) => ({
  id: t.id,
  taskKey: t.taskKey,
  action: t.action,
  dependsOn: t.dependsOn,
  deviceId: t.deviceId,
  status: t.status,
  currentWp: t.currentWp,
  totalWp: t.totalWp,
});

const { reducer: activeMissionsReducer, actions: activeMissionsActions } = createSlice({
  name: 'activeMissions',
  initialState: {
    // { [missionId]: { id, name, status, uav, initTime, tasks: { [taskId]: { id, taskKey, action, dependsOn, deviceId, status, currentWp, totalWp, ... } } } }
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
          tasks: {},
        };
      }
    },

    setTasks(state, action) {
      // Merge tasks from GET /api/missions/tasks — payload: MissionTask[]
      for (const t of action.payload) {
        const mission = state.items[t.missionId];
        if (!mission) continue;
        mission.tasks[t.id] = { ...mission.tasks[t.id], ...taskFields(t) };
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
        tasks: existing?.tasks ?? {},
      };
    },

    upsertTask(state, action) {
      // From WS taskUpdated: either the MissionTask DB row, or a diagnostics-only
      // update from wpTracking ({ id, missionId, deviceId, status, currentWp, totalWp }
      // + anomalies/wpEstimate/confidence) that lacks taskKey/action/dependsOn — so
      // merge over the stored task instead of replacing it.
      const t = action.payload;
      const { anomalies = [], wpEstimate = null, confidence = null } = t;
      // If mission is unknown, create a placeholder — SocketController will fetch full data.
      // Its own status isn't known yet (that arrives via a separate missionUpdated event).
      if (!state.items[t.missionId]) {
        state.items[t.missionId] = {
          id: t.missionId,
          name: `Mission ${t.missionId}`,
          status: 'running',
          uav: [],
          initTime: null,
          endTime: null,
          tasks: {},
        };
      }
      const tasks = state.items[t.missionId].tasks;
      const known = Object.fromEntries(
        Object.entries(taskFields(t)).filter(([, v]) => v !== undefined),
      );
      tasks[t.id] = { ...tasks[t.id], ...known, anomalies, wpEstimate, confidence };
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

// A mission can be commanded only when at least one task is LOADED (loaded but
// not yet commanded — the manual flow's roots). Tasks already running are skipped.
const isCommandable = (mission) =>
  Object.values(mission?.tasks ?? {}).some((t) => t.status === 'loaded');

/**
 * Resolves which missionId the user can command right now: the selected mission
 * (activeMissions.selectedMissionId), but only if it still has LOADED tasks.
 * A successful manual load selects the created mission; after a page refresh the
 * user picks it again from the tracking panel. Returns null otherwise.
 */
export const getCommandableMissionId = (state) => {
  const { items, selectedMissionId } = state.activeMissions;
  if (selectedMissionId != null && isCommandable(items[selectedMissionId]))
    return selectedMissionId;

  return null;
};
