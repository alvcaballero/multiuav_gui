import { useState } from 'react';
import { CommandType } from '../../common/util/commandTypes';

const missionTitle = (mission) => mission.name || `Mission ${mission.id}`;

/**
 * Pause/Resume/Stop/Abort for the device's open mission task. An action is null when the device
 * doesn't support the command behind it. The server has no "paused" task status, so `paused`
 * is local: this page knows because it sent the pause.
 */
export const useMissionActions = ({ missionInfo, capabilities, send, askConfirm, deviceName }) => {
  const [pausedTaskId, setPausedTaskId] = useState(null);

  const task = missionInfo?.task;
  const paused = task != null && pausedTaskId === task.id;
  const running = task?.status === 'running' && !paused;
  const canReturn = capabilities.has(CommandType.RETURN_HOME);
  const canStop = capabilities.has(CommandType.STOP_MISSION);

  const pause = (label) =>
    send(CommandType.PAUSE_MISSION, {}, label).then((ok) => ok && setPausedTaskId(task.id));

  const abort = () =>
    askConfirm({
      title: 'Abort mission?',
      message: `${deviceName} will stop “${missionTitle(missionInfo.mission)}”${
        canReturn ? ' and return to launch' : ''
      }.`,
      cancelLabel: 'Keep flying',
      confirmLabel: canReturn ? 'Abort and return home' : 'Abort',
      onConfirm: async () => {
        if (canStop) await send(CommandType.STOP_MISSION);
        if (canReturn) await send(CommandType.RETURN_HOME, {}, 'Mission aborted — returning home');
      },
    });

  const actions = {
    pause: capabilities.has(CommandType.PAUSE_MISSION) ? () => pause() : null,
    resume: capabilities.has(CommandType.RESUME_MISSION)
      ? () => send(CommandType.RESUME_MISSION).then((ok) => ok && setPausedTaskId(null))
      : null,
    stop: canStop ? () => send(CommandType.STOP_MISSION) : null,
    abort: canStop || canReturn ? abort : null,
  };

  // Taking manual control pauses a running mission first.
  const pauseForControl = () =>
    running && actions.pause ? pause('Control taken — mission paused') : Promise.resolve(false);

  return { actions, paused, pauseForControl };
};
