import { useImperativeHandle, useMemo, useRef } from 'react';
import { useSelector } from 'react-redux';

import { SocketCameraCanvas } from './SocketCameraCanvas';
import { CameraMediaMTXPlayer } from './CameraMediaMTXPlayer';

const hostname = window.location.hostname;

export const customEqual = (oldValue, newValue) => {
  return (
    oldValue?.camera === newValue?.camera &&
    oldValue?.ip === newValue?.ip &&
    oldValue?.name === newValue?.name
  );
};

// Selects between the two camera transports for a device — raw Websocket JPEG
// frames (SocketCameraCanvas) or WebRTC/MediaMTX (CameraMediaMTXPlayer) — and
// exposes an imperative capture() so a chrome layer (e.g. CameraDevicePanel)
// can grab the current frame without knowing which transport is active.
const CameraDevice = ({ deviceId, cameraIndex = 0, ref }) => {
  const device = useSelector((state) => state.devices.items[deviceId], customEqual);
  const videoRef = useRef(null);
  const socketCanvasRef = useRef(null);

  const { type, cameraSrc, srcIp } = useMemo(() => {
    const cam = device?.camera?.[cameraIndex];
    if (!cam) return { type: 'Websocket', cameraSrc: '', srcIp: '' };
    if (cam.type === 'WebRTC') {
      return { type: 'WebRTC', cameraSrc: `${device.name}_${cam.source}`, srcIp: hostname };
    }
    return { type: cam.type, cameraSrc: cam.source, srcIp: device.ip };
  }, [device, cameraIndex]);

  useImperativeHandle(
    ref,
    () => ({
      cameraSrc,
      capture: () => {
        if (type === 'Websocket') {
          return socketCanvasRef.current?.toDataURL('image/jpeg') ?? null;
        }
        if (videoRef.current) {
          const canvas = document.createElement('canvas');
          canvas.width = videoRef.current.videoWidth;
          canvas.height = videoRef.current.videoHeight;
          canvas.getContext('2d').drawImage(videoRef.current, 0, 0);
          return canvas.toDataURL('image/jpeg');
        }
        return null;
      },
    }),
    [type, cameraSrc],
  );

  if (!device) return null;

  return type === 'Websocket' ? (
    <SocketCameraCanvas ref={socketCanvasRef} deviceId={deviceId} />
  ) : (
    <CameraMediaMTXPlayer
      src={`http://${srcIp}:8889/${cameraSrc}`}
      videoRef={videoRef}
      label={`${device.name} camera feed`}
    />
  );
};

export default CameraDevice;
