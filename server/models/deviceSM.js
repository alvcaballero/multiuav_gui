//https://stately.ai/docs/editor-states-and-transitions
// https://dev.to/davidkpiano/you-don-t-need-a-library-for-state-machines-k7h
import { createMachine, createActor, fromPromise, assign } from 'xstate';
import { commandsController } from '../controllers/commands.js';
//import { missionModel } from './mission.js';
import { dateString, addTime, GetLocalTime, sleep } from '../common/utils.js';
import { missionSMModel } from './missionSM.js';
import { missionController } from '../controllers/mission.js';

const LoadMissionSM = async (context) => {
  console.log('service load mission');
  try {
    let mission = await MissionController.getMissionRoute(context.missionId);
    let missionPlan = mission.mission;
    console.log(mission);
    console.log(missionPlan);
    let response = await commandsController.sendCommandDevice({
      deviceId: context.uavId,
      type: 'loadMission',
      attributes: missionPlan.route,
    });

    console.log('-----response in SM');
    console.log(response);
    if (response.state == 'success') {
      console.log('----- success in SM');
      return response; // Resolve with the response
    } else {
      throw new Error('Problem send Mission');
    }
  } catch (error) {
    throw error; // Reject with the error
  }
};

const CommandMissionSM = async (context) => {
  console.log('service command mission');
  try {
    sleep(2000);
    //console.log(mission);
    let response = await commandsController.sendCommandDevice({
      deviceId: context.uavId,
      type: 'commandMission',
    });

    console.log('-----response in SM');
    console.log(response);
    if (response.state == 'success') {
      console.log('----- success in SM');
      return response; // Resolve with the response
    } else {
      throw new Error('Problem send Command ');
    }
  } catch (error) {
    throw error; // Reject with the error
  }
};

const CommandDownload = async (context) => {
  console.log('service download files from Autopilot ');
  let resp = await MissionController.finishMission(context.missionId, context.uavId);
  let mymission = await MissionController.getMissionRoute(context.missionId);
  //let missionPlan = mission.mission;

  console.log('mission command download');
  console.log(mymission);
  let myInitTime = dateString(GetLocalTime(mymission['initTime']));
  let myFinishTime = dateString(addTime(GetLocalTime(new Date()), 10));
  console.log(myInitTime + '---' + myFinishTime);
  try {
    let response = await commandsController.sendCommandDevice({
      deviceId: context.uavId,
      type: 'CameraFileDownload',
      attributes: {
        downloadCnt: 0,
        initDate: myInitTime,
        FinishDate: myFinishTime,
      },
    });
    console.log('-----response in Download');
    console.log(response);
    if (response.state == 'success') {
      console.log('----- success in download');
      return response; // Resolve with the response
    } else {
      console.log('Problem download Mission');
      //throw new Error('Problem download Mission');
    }
  } catch (error) {
    throw error; // Reject with the error
  }
};

const DownloadGCS = async (context) => {
  console.log('Download files from UAV');
  let result = await MissionController.updateFiles(context.missionId, context.uavId, context.routeId);
  console.log(`Download files from UAV id ${context.uavId}`);
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
export const machine = createMachine(
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
          input: ({ context: { uavId } }) => ({ uavId }),
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
        console.log('action Assing');
        console.log(event);
        return {
          uavId: event.value.uavId,
          missionId: event.value.missionId,
        };
      }),
      DeleteSM: ({ context, event }, params) => {
        console.log('delete state machine');
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
