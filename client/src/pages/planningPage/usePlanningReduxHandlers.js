import { useCallback } from 'react';

import { sessionActions } from '../../store';
import { manageLocationPoints } from '../../services/planningService';

export const usePlanningReduxHandlers = ({ dispatch, markers, SendTask, sendTaskRef }) => {
  const setMarkersBase = useCallback(
    (value, meta = {}) => {
      dispatch(sessionActions.updateMarker({ ...markers, bases: value }));

      if (meta.meth === 'del' && meta.id) {
        const newAssignments = (SendTask.assignments || []).filter((a) => a.baseId !== meta.id);
        dispatch(sessionActions.updatePlanning({ ...SendTask, assignments: newAssignments }));
      }
    },
    [dispatch, markers, SendTask],
  );

  const setMarkersElements = useCallback(
    (value) => dispatch(sessionActions.updateMarker({ ...markers, elements: value })),
    [dispatch, markers],
  );

  const SetMapMarkers = useCallback(
    (value) => dispatch(sessionActions.updateMarker(value)),
    [dispatch],
  );

  const setLocations = useCallback(
    (value) => dispatch(sessionActions.updatePlanning({ ...SendTask, loc: value })),
    [dispatch, SendTask],
  );

  const addLocations = useCallback(
    (value) => {
      const newLoc = manageLocationPoints(
        structuredClone(SendTask.loc),
        value,
        SendTask.objetivo.type,
      );
      dispatch(sessionActions.updatePlanning({ ...SendTask, loc: newLoc }));
    },
    [dispatch, SendTask],
  );

  const setBaseSettings = useCallback(
    (assignments) => dispatch(sessionActions.updatePlanning({ ...SendTask, assignments })),
    [dispatch, SendTask],
  );

  const updateObjetive = useCallback(
    (newObjetive) => {
      const myTask = structuredClone(sendTaskRef.current);
      myTask.objetivo = newObjetive;
      if (newObjetive.type !== sendTaskRef.current.objetivo.type) myTask.loc = [];
      dispatch(sessionActions.updatePlanning(myTask));
    },
    [dispatch, sendTaskRef],
  );

  return {
    setMarkersBase,
    setMarkersElements,
    SetMapMarkers,
    setLocations,
    addLocations,
    setBaseSettings,
    updateObjetive,
  };
};
