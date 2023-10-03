import { useMemo } from 'react';

export default () =>
  useMemo(
    () => ({
      custom: [
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      positionPeriodic: [
        {
          key: 'frequency',
          name: 'commandFrequency',
          type: 'number',
        },
      ],
      setTimezone: [
        {
          key: 'timezone',
          name: 'commandTimezone',
          type: 'string',
        },
      ],
      sendSms: [
        {
          key: 'phone',
          name: 'commandPhone',
          type: 'string',
        },
        {
          key: 'message',
          name: 'commandMessage',
          type: 'string',
        },
      ],
      message: [
        {
          key: 'message',
          name: 'commandMessage',
          type: 'string',
        },
      ],
      sendUssd: [
        {
          key: 'phone',
          name: 'commandPhone',
          type: 'string',
        },
      ],
      sosNumber: [
        {
          key: 'index',
          name: 'commandIndex',
          type: 'number',
        },
        {
          key: 'phone',
          name: 'commandPhone',
          type: 'string',
        },
      ],
      silenceTime: [
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      setPhonebook: [
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      voiceMessage: [
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      outputControl: [
        {
          key: 'index',
          name: 'commandIndex',
          type: 'number',
        },
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      voiceMonitoring: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      setAgps: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      setIndicator: [
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      configuration: [
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      setConnection: [
        {
          key: 'server',
          name: 'commandServer',
          type: 'string',
        },
        {
          key: 'port',
          name: 'commandPort',
          type: 'number',
        },
      ],
      setOdometer: [
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      modePowerSaving: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      modeDeepSleep: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      alarmGeofence: [
        {
          key: 'radius',
          name: 'commandRadius',
          type: 'number',
        },
      ],
      alarmBattery: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      alarmSos: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      alarmRemove: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      alarmClock: [
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      alarmSpeed: [
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      alarmFall: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      alarmVibration: [
        {
          key: 'data',
          name: 'commandData',
          type: 'string',
        },
      ],
      ResumeMission: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      StopMission: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      Gimbal: [
        {
          key: 'roll',
          name: 'Roll',
          type: 'number',
        },
        {
          key: 'pitch',
          name: 'Pitch',
          type: 'number',
        },
        {
          key: 'yaw',
          name: 'Yaw',
          type: 'number',
        },
      ],
      GimbalPitch: [
        {
          key: 'Pitch',
          name: 'Pitch',
          type: 'number',
        },
      ],
      SincroniseFiles: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
    }),
    []
  );
