import { useDispatch } from 'react-redux';
import { missionActions } from '../../store';

const DEFAULT_UAV_TYPE = 'dji_M210_noetic';

const useWaypoint = (routeIndex, wpIndex, uavType) => {
  const resolvedUavType = uavType || DEFAULT_UAV_TYPE;
  const dispatch = useDispatch();

  const updateField = (field, value) =>
    dispatch(missionActions.updateWaypoint({ routeIndex, wpIndex, field, value }));

  const updatePos = (pos) =>
    dispatch(missionActions.updateWaypointPos({ routeIndex, wpIndex, pos }));

  const updateAction = (actionKey, value) =>
    dispatch(missionActions.updateWaypointAction({ routeIndex, wpIndex, actionKey, value }));

  const removeAction = (actionKey) =>
    dispatch(missionActions.removeWaypointAction({ routeIndex, wpIndex, actionKey }));

  const addAction = async (actionId, onDone) => {
    const endpoint = `/api/category/actions/${resolvedUavType}`;
    const response = await fetch(endpoint);
    if (!response.ok) {
      console.error('Failed to fetch actions:', await response.text());
      return;
    }
    const commands = await response.json();
    const cmd = commands.find((c) => c.id == actionId);
    if (!cmd) return;

    // param present → numeric value (0), otherwise boolean flag (true)
    dispatch(missionActions.addWaypointAction({ routeIndex, wpIndex, actionKey: cmd.name, value: cmd.param ? 0 : true }));
    onDone?.();
  };

  const copy = () => dispatch(missionActions.copyWaypoint({ routeIndex, wpIndex }));
  const remove = () => dispatch(missionActions.deleteWaypoint({ routeIndex, wpIndex }));
  const move = (direction) => dispatch(missionActions.moveWaypointOrder({ routeIndex, wpIndex, direction }));

  return { updateField, updatePos, updateAction, removeAction, addAction, copy, remove, move };
};

export default useWaypoint;
