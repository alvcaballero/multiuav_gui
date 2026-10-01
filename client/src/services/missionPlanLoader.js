import { missionActions } from '../store';
import { map } from '../map/core/mapInstance';

const flyToMission = (missionData) => {
  const firstWp = (missionData?.route ?? missionData?.tasks)?.[0]?.wp?.[0];
  if (!firstWp || !map) return;
  const [lat, lon] = Array.isArray(firstWp.pos) ? firstWp.pos : [firstWp.pos.lat, firstWp.pos.lon];
  map.easeTo({ center: [lon, lat], zoom: Math.max(map.getZoom(), 14) });
};

/**
 * Fetches a MissionPlan by id and loads it into the editor (state.mission) of
 * THIS client only, flying the map to its first waypoint. Unlike
 * GET /api/missions/plans/show/:id, nothing is broadcast to other clients.
 * @param {number} planId
 * @param {(action: any) => void} dispatch
 * @param {string} [name] mission name to show in the editor; defaults to the plan's name
 * @returns {Promise<boolean>} whether the plan was loaded
 */
export const loadPlanToEditor = async (planId, dispatch, name) => {
  try {
    const planRes = await fetch(`/api/missions/plans/${planId}`);
    if (!planRes.ok) return false;
    const plan = await planRes.json();
    const missionData = plan.missionData;
    if (!missionData) return false;

    // Plans are either task graphs (tasks[], v4) or the current route format; tag
    // route[] plans '3' so updateMission parses them with RuteConvert, not the legacy parser.
    const version = missionData.tasks ? '4' : '3';
    dispatch(
      missionActions.updateMission({
        ...missionData,
        version,
        name: name ?? plan.name ?? missionData.name ?? `plan ${plan.id}`,
      }),
    );
    flyToMission(missionData);
    return true;
  } catch {
    // fetch failed — silently skip map load
    return false;
  }
};

/**
 * Fetches a manual mission's plan (Mission → MissionPlan) and loads it into the
 * editor via loadPlanToEditor. Shared by MissionTrackingPanel.handleSelect (user
 * clicks a mission) and the initial auto-select of the most recent active
 * mission (SocketController).
 * @param {number} missionId
 * @param {(action: any) => void} dispatch
 */
export const loadMissionPlanToEditor = async (missionId, dispatch) => {
  try {
    const res = await fetch(`/api/missions?id=${missionId}`);
    if (!res.ok) return;
    const record = await res.json();
    const mission = Array.isArray(record) ? record[0] : record;
    const planId = mission?.planId;
    if (!planId) return;

    await loadPlanToEditor(planId, dispatch, mission?.name);
  } catch {
    // fetch failed — silently skip map load
  }
};
