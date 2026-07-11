//https://stately.ai/docs/editor-states-and-transitions
// https://dev.to/davidkpiano/you-don-t-need-a-library-for-state-machines-k7h
import { createMachine, createActor, fromPromise, assign } from 'xstate';
import { commandsController } from '../../controllers/commands.js';
import { dateString, addTime, GetLocalTime, sleep } from '../../common/utils.js';
import { missionSMModel } from './missionSM.js';
import { missionController } from '../../controllers/mission.js';
import { missionLogger as logger } from '../../common/logger.js';
import { ROUTE_STATUS } from '../../config/status.js';

const LoadMissionSM = async (context) => {
  logger.info('service load mission');
  let mission = await missionController.getMissionRoute(context.missionId);
  let missionPlan = mission.mission;
  logger.debug(`LoadMissionSM mission: ${JSON.stringify(missionPlan)}`);
  // Pass the FULL mission; loadMissionToDevice extracts this UAV's own route.
  let response = await commandsController.sendCommandDevice({
    deviceId: context.uavId,
    type: 'loadMission',
    attributes: missionPlan,
  });

  logger.debug(`LoadMissionSM response: ${JSON.stringify(response)}`);
  if (response.state == 'success') {
    logger.info('LoadMissionSM success');
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
    // initMission creates the MissionRoute in INIT; promote it to COMMANDED so
    // missionWpTracking.checkProgress starts tracking waypoint progress.
    await missionController.editRoute({
      missionId: context.missionId,
      deviceId: context.uavId,
      status: ROUTE_STATUS.COMMANDED,
    });
    return response; // Resolve with the response
  } else {
    throw new Error('Problem send Command ');
  }
};

const CommandDownload = async (context) => {
  logger.info('service download files from Autopilot');
  let resp = await missionController.finishMission(context.missionId, context.uavId);
  let mymission = await missionController.getMissionRoute(context.missionId);

  logger.debug(`CommandDownload mission: ${JSON.stringify(mymission)}`);
  let myInitTime = dateString(GetLocalTime(mymission['initTime']));
  let myFinishTime = dateString(addTime(GetLocalTime(new Date()), 10));
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

const DownloadGCS = async (context) => {
  logger.info(`Download files from UAV id ${context.uavId}`);
  let result = await missionController.updateFiles(context.missionId, context.uavId, context.routeId);
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
    context: { uavId: 1, missionId: 1, routeId: 1 },
    initial: 'Initial state',
    states: {
      'Initial state': {
        on: {
          ChangeId: {
            target: 'LoadMission',
            actions: assign(({ event }) => event.value),
          },
          loadMission: { target: 'LoadMission' },
        },
      },
      LoadMission: {
        invoke: {
          src: fromPromise(({ input }) => LoadMissionSMPromise(input)),
          input: ({ context: { uavId, missionId } }) => ({ uavId, missionId }),
          onDone: [{ target: 'Commadmission' }],
          onError: [{ target: 'resetUAV' }],
        },
        on: {
          commandMission: { target: 'Commadmission' },
          'response fail': { target: 'resetUAV' },
        },
      },
      Commadmission: {
        invoke: {
          src: fromPromise(({ input }) => CommandMissionSMPromise(input)),
          input: ({ context: { uavId, missionId } }) => ({ uavId, missionId }),
          onDone: [{ target: 'RunningMission' }],
          onError: [{ target: 'resetUAV' }],
        },
        on: {
          'response ok': { target: 'RunningMission' },
          'response fail': { target: 'resetUAV' },
        },
      },
      resetUAV: {
        on: {
          'confirm reset': { target: 'UAVready' },
        },
        after: {
          10000: { target: 'END' },
        },
      },
      RunningMission: {
        on: {
          downloadFilesUAV: { target: 'UAVDownloadFiles' },
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
          input: ({ context: { uavId, missionId } }) => ({ uavId, missionId }),
        },
        on: {
          downloadFilesGCS: { target: 'DownloadFilesGCS' },
        },
      },
      return2home: {
        on: {
          landing: { target: 'UAVDownloadFiles' },
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
          input: ({ context: { uavId, missionId, routeId } }) => ({ uavId, missionId, routeId }),
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
              missionSMModel.DeleteActor(context.uavId);
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
      DeleteSM: ({ context, event }, params) => {
        logger.info(`delete state machine for uavId ${context.uavId}`);
        missionSMModel.DeleteActor(context.uavId);
      },
    },
    actors: {},
    guards: {},
    delays: {
      TIMEOUT: 1000,
    },
  }
);
