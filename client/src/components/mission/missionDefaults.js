// Shared fallback for the mission planning UI when a route has no UAV assigned yet.
//
// The server resolves any unknown/serviceless category to the catalog's 'default'
// profile (unfiltered baseline), so this placeholder just needs to be a value the
// /api/category/* path segments accept. Keep it in ONE place so RouteRouteList and
// useWaypoint can't drift apart (they used to hardcode dji_M300 vs dji_M210_noetic).
export const DEFAULT_UAV_TYPE = '__no_uav__';

// Suggestions for a task's `action`. It is semantic only (UI, logs, the LLM): every
// task still flies as waypoints, so any other text is accepted too. 'ROUTE' is what
// the server assigns to a legacy route with no action.
export const TASK_ACTION_SUGGESTIONS = [
  'ROUTE',
  'NAVIGATE',
  'TAKEOFF_AND_SURVEY',
  'INSPECTION',
  'RETURN',
];
