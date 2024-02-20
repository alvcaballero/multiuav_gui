//https://stately.ai/docs/editor-states-and-transitions
// https://dev.to/davidkpiano/you-don-t-need-a-library-for-state-machines-k7h
import { createMachine, createActor, fromPromise, assign } from 'xstate';
import { commandsModel } from '../models/commands.js';
import { missionModel } from '../models/mission.js';

const listSM = {}; // lista de acots maquinas de estados por id de UAV

const LoadMissionSM = async (context) => {
  console.log('service load mission');
  try {
    let mission = missionModel.getmission(context.missionId);
    //console.log(mission);
    let response = await commandsModel.loadmissionDevice(context.uavId, mission.route);
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
    //console.log(mission);
    let response = await commandsModel.commandMissionDevice(context.uavId);
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
  console.log('service load mission');
  let mymission = missionModel.getmissionValue(context.missionId);
  let myinitTime = mymission['initTime'].toISOString().slice(0, -8).replace('T', ' ');
  let myfinishTime = new Date().toISOString().slice(0, -8).replace('T', ' ');
  console.log(myinitTime + '---' + myfinishTime);
  try {
    let response = await commandsModel.sendCommand(context.uavId, 'CameraFileDownload', {
      downloadCnt: 0,
      initDate: myinitTime,
      FinishDate: myfinishTime,
    });
    console.log('-----response in Download');
    console.log(response);
    if (response.state == 'success') {
      console.log('----- success in download');
      return response; // Resolve with the response
    } else {
      throw new Error('Problem send Mission');
    }
  } catch (error) {
    throw error; // Reject with the error
  }
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

// https://stately.ai/docs/invoke
const machine = createMachine(
  {
    id: 'GCS-UAV',
    context: { uavId: 1, missionId: 1 },
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
          onDone: [{ target: 'RuningMission' }],
          onError: [{ target: 'resetUAV' }],
        },
        on: {
          'response ok': { target: 'RuningMission' },
          'response fail': { target: 'resetUAV' },
        },
      },
      resetUAV: {
        on: {
          'confirm reset': { target: 'UAVready' },
        },
      },
      RuningMission: {
        on: {
          finishMission: { target: 'DownloadFiles' },
          cancelMission: { target: 'return2home' },
          stopMission: { target: 'stopMission' },
        },
      },
      UAVready: {
        on: {
          timer: { target: 'LoadMission' },
        },
      },
      DownloadFiles: {
        invoke: {
          src: fromPromise(({ input }) => CommandDownloadPromise(input)),
          input: ({ context: { uavId, missionId } }) => ({ uavId, missionId }),
          onDone: [{}],
          onError: [{}],
        },
        on: {
          downloadFiles: { target: 'DownloadFilesGCS' },
        },
      },
      return2home: {
        on: {
          landing: { target: 'DownloadFiles' },
        },
      },
      stopMission: {
        on: {
          'resume mission response ok': { target: 'RuningMission' },
          'Event 2': { target: 'return2home' },
        },
      },
      DownloadFilesGCS: {
        on: {
          'Event name': { target: 'END' },
        },
      },
      END: { type: 'final' },
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
      fetch_planning: ({ context, event }) => {
        console.log(event.value);
      },
    },
    actors: {
      load_mission: async (context) => {
        console.log('service load mission');
        let mission = missionModel.getmission(context.missionId);
        let response = await commandsModel.loadmissionDevice(context.uavId, mission);
        if (response.success == 'success') {
          return response;
        }
        return null;
      },
      Command_mission: async ({ context }) => {
        console.log('service command mission');
        let response = await commandsModel.commandMissionDevice(context.uavId);
        if (response.success == 'success') {
          return response;
        }
        return null;
      },
    },
    guards: {},
    delays: {},
  }
);

export class missionSMModel {
  static createActorMission(uavId = 0, missionId = 0) {
    listSM[uavId] = createActor(machine).start();
    listSM[uavId].subscribe((state) => {
      console.log('state machine');
      console.log('Value:', state.context);
    });
    listSM[uavId].send({ type: 'ChangeId', value: { uavId, missionId } });
    return true;
  }

  static get_status(id) {
    return listSM[id].states;
  }

  static load_mission(id) {
    listSM[id].send({ type: 'loadMission' });
  }
  static command_mission(id) {
    listSM[id].send({ type: 'commandMission' });
  }
  static finishMission(id) {
    listSM[id].send({ type: 'DownloadFiles' });
  }
  static DownloadFiles(id) {
    listSM[id].send({ type: 'DownloadFilesGCS' });
  }
}
