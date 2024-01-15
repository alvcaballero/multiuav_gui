//https://stately.ai/docs/editor-states-and-transitions
// https://dev.to/davidkpiano/you-don-t-need-a-library-for-state-machines-k7h
import { createMachine, createActor } from 'xstate';
const listSM = {}; // lista de acots maquinas de estados por id de UAV

const machine = createMachine(
  {
    id: 'GCS-UAV',
    context: { uav_id: 1 },
    initial: 'Initial state',
    states: {
      'Initial state': {
        on: {
          'recibe sendtaks': {
            target: 'Planing',
          },
        },
      },
      Planing: {
        entry: {
          type: 'fetch_planing',
        },
        on: {
          'recibe planing': {
            target: 'Loadmission',
          },
        },
      },
      Loadmission: {
        entry: {
          type: 'load_mission',
        },
        on: {
          'response ok': {
            target: 'Commadmission',
          },
          'response fail': {
            target: 'resetUAV',
          },
        },
      },
      Commadmission: {
        on: {
          'response ok': {
            target: 'Runingmission',
          },
          'response fail': {
            target: 'resetUAV',
          },
        },
      },
      resetUAV: {
        on: {
          'confime reset': {
            target: 'UAVready',
          },
        },
      },
      Runingmission: {
        on: {
          finishmission: {
            target: 'DownloadFiles',
          },
          cancel: {
            target: 'return2home',
          },
          stop: {
            target: 'stopmission',
          },
        },
      },
      UAVready: {
        on: {
          timer: {
            target: 'Loadmission',
          },
        },
      },
      DownloadFiles: {
        on: {
          downloadFiles: {
            target: 'DonwloadFilesGCS',
          },
        },
      },
      return2home: {
        on: {
          landing: {
            target: 'DownloadFiles',
          },
        },
      },
      stopmission: {
        on: {
          'resume mission response ok': {
            target: 'Runingmission',
          },
          'Event 2': {
            target: 'return2home',
          },
        },
      },
      DonwloadFilesGCS: {
        on: {
          'Event name': {
            target: 'END',
          },
        },
      },
      END: {
        type: 'final',
      },
    },
  },
  {
    actions: {
      load_mission: ({ context, event }) => {
        console.log(event.data);
      },
      fetch_planing: ({ context, event }) => {},
    },
    actors: {},
    guards: {},
    delays: {},
  }
);

export class missionSMModel {
  static createActorMission(id) {
    //--- poner un estado anterior donde se asigne el id

    //const myMachine = machine.withContext({
    //  uav_id: id,
    //});

    //listSM[id] = createActor(myMachine);
    listSM[id] = createActor(machine);
    listSM[id].subscribe((snapshot) => {
      console.log('Value:', snapshot.value);
    });
    listSM[id].start();
    return true;
  }

  static get_status() {
    return listSM[id].states;
  }

  static load_mission() {
    // get  the  mission plan with id of  uav
    //  load sendCommand of command model
    // configure for get the answer and the andwer of this change the actor of the state machine,
  }
  static command_mission() {
    // get  the  mission plan with id of  uav
    //  load sendCommand of command model
    // configure for get the answer and the andwer of this change the actor of the state machine,
  }
}
