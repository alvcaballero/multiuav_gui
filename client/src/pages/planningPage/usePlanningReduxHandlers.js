import { useCallback, useMemo } from 'react';

import { sessionActions } from '../../store';
import { manageLocationPoints } from '../../services/planningService';
import { useMarkerTypes } from '../../hooks/useMarkerTypes';

export const usePlanningReduxHandlers = ({
  dispatch,
  markers,
  missionRequest,
  missionRequestRef,
}) => {
  const { types: markerTypes } = useMarkerTypes();
  // 'Power Tower' is the default type for a line drawn via addLocations
  // (path-object/object/point) — resolved by name since the catalog's PK is
  // an opaque autoincrement id, not a stable slug.
  const powerTowerTypeId = useMemo(
    () => markerTypes.find((type) => type.name === 'Power Tower')?.id,
    [markerTypes],
  );

  const setMarkersBase = useCallback(
    (value, meta = {}) => {
      dispatch(sessionActions.updateMarker({ ...markers, bases: value }));

      if (meta.meth === 'del' && meta.id) {
        const newAssignments = (missionRequest.assignments || []).filter(
          (a) => a.baseId !== meta.id,
        );
        dispatch(sessionActions.updatePlanning({ ...missionRequest, assignments: newAssignments }));
      }
    },
    [dispatch, markers, missionRequest],
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
    (value) => dispatch(sessionActions.updatePlanning({ ...missionRequest, loc: value })),
    [dispatch, missionRequest],
  );

  const addLocations = useCallback(
    (value) => {
      const newLoc = manageLocationPoints(
        structuredClone(missionRequest.loc),
        value,
        missionRequest.objetivo.type,
        powerTowerTypeId,
      );
      dispatch(sessionActions.updatePlanning({ ...missionRequest, loc: newLoc }));
    },
    [dispatch, missionRequest, powerTowerTypeId],
  );

  const setBaseSettings = useCallback(
    (assignments) => dispatch(sessionActions.updatePlanning({ ...missionRequest, assignments })),
    [dispatch, missionRequest],
  );

  const updateObjetive = useCallback(
    (newObjetive) => {
      const nextRequest = structuredClone(missionRequestRef.current);
      nextRequest.objetivo = newObjetive;
      if (newObjetive.type !== missionRequestRef.current.objetivo.type) nextRequest.loc = [];
      dispatch(sessionActions.updatePlanning(nextRequest));
    },
    [dispatch, missionRequestRef],
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
