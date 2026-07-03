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
      setTimezone: [
        {
          key: 'timezone',
          name: 'commandTimezone',
          type: 'string',
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
          key: 'pitch',
          name: 'Pitch',
          type: 'number',
        },
      ],
      treat: [
        {
          key: 'enable',
          name: 'commandEnable',
          type: 'boolean',
        },
      ],
      setupcamera: [
        {
          key: 'cameraType',
          name: 'cameraType',
          type: 'number',
        },
        {
          key: 'start',
          name: 'start',
          type: 'number',
        },
      ],
      navigateToPose: [
        {
          key: 'x',
          name: 'commandX',
          type: 'number',
        },
        {
          key: 'y',
          name: 'commandY',
          type: 'number',
        },
        {
          key: 'z',
          name: 'commandZ',
          type: 'number',
        },
      ],
      CameraFileDownload: [
        {
          key: 'startDate',
          name: 'commandStartDate',
          type: 'string',
        },
        {
          key: 'endDate',
          name: 'commandEndDate',
          type: 'string',
        },
      ],
    }),
    []
  );
