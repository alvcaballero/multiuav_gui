import { useEffect, useRef, useState } from 'react';
import { Box } from '@mui/material';

import novideo from '../../resources/images/placeholder.jpg';

// Reads camera frames straight off window.websocket, bypassing Redux entirely:
// each JSON tick's base64 frame is decoded off the main thread (createImageBitmap)
// and painted via requestAnimationFrame, so frame arrival never triggers a React render.
export const SocketCameraCanvas = ({ deviceId, ref }) => {
  const latestBitmapRef = useRef(null);
  const [hasFrame, setHasFrame] = useState(false);

  useEffect(() => {
    setHasFrame(false);
    let cancelled = false;
    let attachedSocket = null;

    const onMessage = (event) => {
      if (typeof event.data !== 'string') return;
      let data;
      try {
        data = JSON.parse(event.data);
      } catch {
        return;
      }
      const frame = data.camera?.find((c) => String(c.deviceId) === String(deviceId));
      if (!frame) return;

      fetch(`data:image/jpeg;base64,${frame.camera}`)
        .then((res) => res.blob())
        .then((blob) => createImageBitmap(blob))
        .then((bitmap) => {
          if (cancelled) {
            bitmap.close();
            return;
          }
          latestBitmapRef.current?.close();
          latestBitmapRef.current = bitmap;
          setHasFrame((prev) => prev || true);
        })
        .catch(() => {});
    };

    let animationFrameId;
    const render = () => {
      // window.websocket gets replaced on reconnect; re-attach when that happens
      // instead of assuming the socket this effect started with is still live.
      if (window.websocket !== attachedSocket) {
        attachedSocket?.removeEventListener('message', onMessage);
        attachedSocket = window.websocket ?? null;
        attachedSocket?.addEventListener('message', onMessage);
      }

      const canvas = ref?.current;
      const bitmap = latestBitmapRef.current;
      if (canvas && bitmap) {
        if (canvas.width !== bitmap.width) canvas.width = bitmap.width;
        if (canvas.height !== bitmap.height) canvas.height = bitmap.height;
        canvas.getContext('2d').drawImage(bitmap, 0, 0);
      }
      animationFrameId = requestAnimationFrame(render);
    };
    render();

    return () => {
      cancelled = true;
      attachedSocket?.removeEventListener('message', onMessage);
      cancelAnimationFrame(animationFrameId);
      latestBitmapRef.current?.close();
      latestBitmapRef.current = null;
    };
  }, [deviceId, ref]);

  return (
    <Box sx={{ position: 'relative', width: '100%', height: '100%' }}>
      {!hasFrame && (
        <img
          src={novideo}
          alt="Device camera feed"
          style={{
            position: 'absolute',
            inset: 0,
            width: '100%',
            height: '100%',
            objectFit: 'contain',
          }}
        />
      )}
      <canvas
        ref={ref}
        style={{
          width: '100%',
          height: '100%',
          objectFit: 'contain',
          display: hasFrame ? 'block' : 'none',
        }}
      />
    </Box>
  );
};
