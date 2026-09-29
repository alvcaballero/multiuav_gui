//https://stately.ai/docs/editor-states-and-transitions
// https://dev.to/davidkpiano/you-don-t-need-a-library-for-state-machines-k7h
import { createMachine, fromPromise, assign } from 'xstate';
import { commandsController } from '../../controllers/commands.js';
import { addTime, sleep } from '../../common/utils.js';
import { missionSMModel } from './missionSM.js';
import { missionController } from '../../controllers/mission.js';
import { missionLogger as logger } from '../../common/logger.js';
import { TASK_STATUS } from '../../config/status.js';
import { normalizeMission, missionForTask } from './taskGraph.js';

const LoadMissionSM = async (context) => {
  logger.info('service load mission');
  const mission = await missionController.getMissionById(context.missionId);
  const task = await missionController.getTask(context.taskId);
  // Only this task goes to the device: ordering between tasks is the GCS's job.
  const taskMission = missionForTask(normalizeMission(mission.mission), task.taskKey);
  if (!taskMission) throw new Error(`task ${task.taskKey} not found in mission ${context.missionId} plan`);
  logger.debug(`LoadMissionSM task mission: ${JSON.stringify(taskMission)}`);
  let response = await commandsController.sendCommandDevice({
    deviceId: context.uavId,
    type: 'loadMission',
    attributes: taskMission,
  });

  logger.debug(`LoadMissionSM response: ${JSON.stringify(response)}`);
  if (response.state == 'success') {
    logger.info('LoadMissionSM success');
    await missionController.editTask({ id: context.taskId, status: TASK_STATUS.LOADED });
    return response; // Resolve with the response
  } else {
    throw new Error('Problem send Mission');
  }
};

const CommandMissionSM = async (context) => {
  logger.info('service command mission');
  sleep(2000);
  let response = await commandsController.sendCommandDevice({
    deviceId: context.uavId,
    type: 'commandMission',
  });

  logger.debug(`CommandMissionSM response: ${JSON.stringify(response)}`);
  if (response.state == 'success') {
    logger.info('CommandMissionSM success');
    // initMission creates the task in INIT; promote it to COMMANDED so
    // missionWpTracking.checkProgress starts tracking waypoint progress.
    await missionController.editTask({ id: context.taskId, status: TASK_STATUS.COMMANDED });
    return response; // Resolve with the response
  } else {
    throw new Error('Problem send Command ');
  }
};

const CommandDownload = async (context) => {
  logger.info('service download files from Autopilot');
  await missionController.finishTask(context.taskId);
  let mymission = await missionController.getMissionById(context.missionId);

  logger.debug(`CommandDownload mission: ${JSON.stringify(mymission)}`);
  // The download service expects UTC 0 timestamps. `Date.toISOString()` emits
  // UTC ISO 8601 with the trailing `Z` straight from the epoch — do NOT run it
  // through GetLocalTime, which corrupts the epoch to fake a local-as-UTC time.
  let myInitTime = new Date(mymission['initTime']).toISOString();
  let myFinishTime = addTime(new Date(), 10).toISOString();
  logger.debug(`CommandDownload time range: ${myInitTime} --- ${myFinishTime}`);
  let response = await commandsController.sendCommandDevice({
    deviceId: context.uavId,
    type: 'CameraFileDownload',
    attributes: {
      startDate: myInitTime,
      endDate: myFinishTime,
    },
  });
  logger.debug(`CommandDownload response: ${JSON.stringify(response)}`);
  if (response.state == 'success') {
    logger.info('CommandDownload success');
    return response; // Resolve with the response
  } else {
    logger.warn('Problem download Mission');
    //throw new Error('Problem download Mission');
  }
};

// A task that is not its device's last open one: close it without downloading, the
// device's last task downloads the whole mission time window at once.
const FinishWithoutDownloadSM = async (context) => {
  await missionController.finishTask(context.taskId);
  await missionController.endTask(context.taskId);
};

// A task whose load/command failed never flew: ERROR, so the scheduler skips its
// dependents instead of leaving it in INIT (where it would be re-dispatched).
const MarkTaskFailedSM = async (context) => {
  await missionController.editTask({
    id: context.taskId,
    status: TASK_STATUS.ERROR,
    errorMessage: `No se pudo ejecutar la tarea en el dispositivo: ${context.failure ?? 'error desconocido'}`,
  });
};

const DownloadGCS = async (context) => {
  logger.info(`Download files from UAV id ${context.uavId}`);
  await missionController.updateTaskFiles(context.taskId);
  return { state: 'success' };
};

const LoadMissionSMPromise = (context) =>
  new Promise((resolve, reject) => {
    LoadMissionSM(context)
      .then((response) => resolve(response))
      .catch((error) => reject(error));
  });

const CommandMissionSMPromise = (context) =>
  new Promise((resolve, reject) => {
    CommandMissionSM(context)
      .then((response) => resolve(response))
      .catch((error) => reject(error));
  });
const CommandDownloadPromise = (context) =>
  new Promise((resolve, reject) => {
    CommandDownload(context)
      .then((response) => resolve(response))
      .catch((error) => reject(error));
  });

const DownloadGCSPromise = (context) =>
  new Promise((resolve, reject) => {
    DownloadGCS(context)
      .then((response) => resolve(response))
      .catch((error) => reject(error));
  });

// https://stately.ai/docs/invoke
export const deviceSM = createMachine(
  {
    id: 'GCS-UAV',
    context: { uavId: 1, missionId: 1, taskId: 1, failure: null },
    initial: 'Initial state',
    states: {
      'Initial state': {
        on: {
          ChangeId: {
            target: 'LoadMission',
            actions: assign(({ event }) => event.value),
          },
          AttachRunning: {
            target: 'RunningMission',
            actions: assign(({ event }) => event.value),
          },
          loadMission: { target: 'LoadMission' },
        },
      },
      LoadMission: {
        invoke: {
          src: fromPromise(({ input }) => LoadMissionSMPromise(input)),
          input: ({ context: { uavId, missionId, taskId } }) => ({ uavId, missionId, taskId }),
          onDone: [{ target: 'Commadmission' }],
          onError: [
            { target: 'resetUAV', actions: assign({ failure: ({ event }) => `load: ${event.error?.message}` }) },
          ],
        },
        on: {
          commandMission: { target: 'Commadmission' },
          'response fail': { target: 'resetUAV' },
        },
      },
      Commadmission: {
        invoke: {
          src: fromPromise(({ input }) => CommandMissionSMPromise(input)),
          input: ({ context: { uavId, missionId, taskId } }) => ({ uavId, missionId, taskId }),
          onDone: [{ target: 'RunningMission' }],
          onError: [
            { target: 'resetUAV', actions: assign({ failure: ({ event }) => `command: ${event.error?.message}` }) },
          ],
        },
        on: {
          'response ok': { target: 'RunningMission' },
          'response fail': { target: 'resetUAV' },
        },
      },
      resetUAV: {
        invoke: {
          src: fromPromise(({ input }) => MarkTaskFailedSM(input)),
          input: ({ context: { taskId, failure } }) => ({ taskId, failure }),
        },
        on: {
          'confirm reset': { target: 'UAVready' },
        },
        after: {
          10000: { target: 'END' },
        },
      },
      RunningMission: {
        on: {
          downloadFilesUAV: { target: 'DecideDownload' },
          cancelMission: { target: 'return2home' },
          stopMission: { target: 'stopMission' },
        },
      },
      UAVready: {
        on: {
          timer: { target: 'LoadMission' },
        },
      },
      UAVDownloadFiles: {
        invoke: {
          src: fromPromise(({ input }) => CommandDownloadPromise(input)),
          input: ({ context: { uavId, missionId, taskId } }) => ({ uavId, missionId, taskId }),
        },
        on: {
          downloadFilesGCS: { target: 'DownloadFilesGCS' },
        },
      },
      // Decided when the task finishes, not when it was dispatched: if a later task of
      // this device got skipped meanwhile, this one is now the last and must download.
      DecideDownload: {
        invoke: {
          src: fromPromise(({ input }) => missionController.shouldDownloadFiles(input.taskId)),
          input: ({ context: { taskId } }) => ({ taskId }),
          onDone: [{ guard: ({ event }) => event.output, target: 'UAVDownloadFiles' }, { target: 'FinishWithoutDownload' }],
          onError: [{ target: 'UAVDownloadFiles' }],
        },
      },
      FinishWithoutDownload: {
        invoke: {
          src: fromPromise(({ input }) => FinishWithoutDownloadSM(input)),
          input: ({ context: { taskId } }) => ({ taskId }),
          onDone: [{ target: 'END' }],
          onError: [{ target: 'END' }],
        },
      },
      return2home: {
        on: {
          landing: { target: 'DecideDownload' },
        },
      },
      stopMission: {
        on: {
          'resume mission response ok': { target: 'RunningMission' },
          'Event 2': { target: 'return2home' },
        },
      },
      DownloadFilesGCS: {
        invoke: {
          src: fromPromise(({ input }) => DownloadGCSPromise(input)),
          input: ({ context: { uavId, missionId, taskId } }) => ({ uavId, missionId, taskId }),
          onDone: [{ target: 'END' }],
        },
        after: {
          60000: { target: 'END' },
        },
        on: {
          FinishMission: { target: 'END' },
        },
      },
      END: {
        after: {
          6000: {
            actions: ({ context }) => {
              missionSMModel.DeleteActor(context.taskId);
            },
          },
        },
      },
    },
  },
  {
    actions: {
      updateUAV: assign(({ event }) => {
        logger.debug(`action updateUAV: ${JSON.stringify(event)}`);
        return {
          uavId: event.value.uavId,
          missionId: event.value.missionId,
        };
      }),
      DeleteSM: ({ context }, _params) => {
        logger.info(`delete state machine for task ${context.taskId}`);
        missionSMModel.DeleteActor(context.taskId);
      },
    },
    actors: {},
    guards: {},
    delays: {
      TIMEOUT: 1000,
    },
  }
);
