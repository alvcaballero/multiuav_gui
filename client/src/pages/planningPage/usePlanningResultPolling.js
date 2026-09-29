import { useEffect } from 'react';

import { missionActions, activeMissionsActions } from '../../store';

const MAX_RETRIES = 12;
const POLLING_INTERVAL = 5000;
const SUCCESS_CODE = 100;

export const usePlanningResultPolling = ({
  requestPlanning,
  setRequestPlanning,
  missionRequestId,
  myhostname,
  dispatch,
}) => {
  useEffect(() => {
    if (requestPlanning >= SUCCESS_CODE || requestPlanning >= MAX_RETRIES) return;

    let cancelled = false;

    const fetchData = async () => {
      try {
        const response = await fetch(`http://${myhostname}:8004/get_plan?IDs=${missionRequestId}`);
        if (!response.ok) throw new Error('Network response was not ok');

        const data = await response.json();
        const planResult = data.results?.[missionRequestId];
        if (cancelled) return;
        if (planResult?.hasOwnProperty('route')) {
          setRequestPlanning(SUCCESS_CODE);
          dispatch(missionActions.updateMission({ ...planResult, version: '3' }));
          // New plan replaces the editor — drop any active selection.
          dispatch(activeMissionsActions.selectMission(null));
          return;
        }
      } catch (error) {
        console.error('Error fetching planning data:', error);
      } finally {
        if (!cancelled) setRequestPlanning((old) => old + 1);
      }
    };

    const intervalId = setInterval(fetchData, POLLING_INTERVAL);

    return () => {
      cancelled = true;
      clearInterval(intervalId);
    };
  }, [requestPlanning, missionRequestId, dispatch, myhostname, setRequestPlanning]);
};
