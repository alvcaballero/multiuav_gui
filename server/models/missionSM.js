//https://stately.ai/docs/editor-states-and-transitions
// https://dev.to/davidkpiano/you-don-t-need-a-library-for-state-machines-k7h
import { createMachine, createActor } from 'xstate';

export const machine = createMachine(
  {
    id: 'GCS-UAV',
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
      load_mission: ({ context, event }) => {},
      fetch_planing: ({ context, event }) => {},
    },
    actors: {},
    guards: {},
    delays: {},
  }
);
export function createActorMission(id) {
  let myactor = createActor(machine);
  myactor.subscribe((snapshot) => {
    console.log('Value:', snapshot.value);
  });
  myactor.start();
  return myactor;
}
