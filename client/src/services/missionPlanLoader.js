import { missionActions } from '../store';
import { map } from '../map/core/MapView';

const flyToMission = (missionData) => {
  const firstWp = missionData?.route?.[0]?.wp?.[0];
  if (!firstWp || !map) return;
  const [lat, lon] = Array.isArray(firstWp.pos) ? firstWp.pos : [firstWp.pos.lat, firstWp.pos.lon];
  map.easeTo({ center: [lon, lat], zoom: Math.max(map.getZoom(), 14) });
};

/**
 * Fetches a manual mission's plan (Mission → MissionPlan) and loads it into the
 * editor (state.mission), flying the map to its first waypoint. Shared by
 * MissionTrackingPanel.handleSelect (user clicks a mission) and the initial
 * auto-select of the most recent active mission (SocketController).
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

    const planRes = await fetch(`/api/missions/plans/${planId}`);
    if (!planRes.ok) return;
    const plan = await planRes.json();
    const missionData = plan.missionData;
    if (!missionData) return;

    // Manual/plan missionData uses the current route format; force version '3'
    // so updateMission parses it with RuteConvert (not the legacy parser).
    dispatch(missionActions.updateMission({ ...missionData, version: '3', name: mission?.name }));
    flyToMission(missionData);
  } catch (_) {
    // fetch failed — silently skip map load
  }
};
