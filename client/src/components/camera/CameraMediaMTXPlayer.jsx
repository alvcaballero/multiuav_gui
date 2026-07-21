import { useEffect, useState } from 'react';
import { Box, CircularProgress, Typography } from '@mui/material';

import { MediaMTXWebRTCReader } from './MediaMTXWebRTCReader';

// antes se usaba el iframe para mostrar el video, pero ahora se usa un elemento de video nativo con WebRTC.
// Esto permite un mejor control y rendimiento en la reproducción del stream de video.
// <iframe
//   src={ttp://127.0.0.1:8889/uav_1_camera_compressed}
//   title={`Camera stream: uav_1`}
//   sandbox="allow-scripts"
// />
export const CameraMediaMTXPlayer = ({ src, videoRef, label }) => {
  const [connection, setConnection] = useState({ src: null, status: 'loading', error: null });
  const loading = connection.src !== src || connection.status === 'loading';
  const error = connection.src === src ? connection.error : null;

  useEffect(() => {
    if (!src) return;

    setConnection({ src, status: 'loading', error: null });

    const whepUrl = src.endsWith('/') ? `${src}whep` : `${src}/whep`;
    const videoEl = videoRef.current;

    const reader = new MediaMTXWebRTCReader({
      url: whepUrl,
      onTrack: (evt) => {
        if (videoRef.current) {
          videoRef.current.srcObject = evt.streams[0];
        }
      },
      onError: (err) => {
        setConnection((c) => {
          if (c.src === src && c.status === 'error') return c;
          console.error('MediaMTX Reader Error:', err);
          return { src, status: 'error', error: err };
        });
      },
    });

    return () => {
      reader.close();
      if (videoEl) videoEl.srcObject = null;
    };
  }, [src, videoRef]);

  return (
    <Box
      sx={{
        position: 'relative',
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        background: 'black',
        overflow: 'hidden',
        width: '100%',
        height: '100%',
      }}
    >
      {loading && (
        <Box
          sx={{
            position: 'absolute',
            top: '50%',
            left: '50%',
            transform: 'translate(-50%, -50%)',
          }}
        >
          <CircularProgress aria-label="Loading…" />
        </Box>
      )}
      {error && !loading && (
        <Box
          sx={{
            position: 'absolute',
            top: '50%',
            left: '50%',
            transform: 'translate(-50%, -50%)',
            textAlign: 'center',
            color: '#888',
          }}
        >
          <Typography variant="caption" display="block">
            STREAM NOT AVAILABLE
          </Typography>
        </Box>
      )}
      <video
        ref={videoRef}
        aria-label={label}
        autoPlay
        muted
        playsInline
        onCanPlay={() => setConnection((c) => (c.src === src ? { ...c, status: 'ready' } : c))}
        style={{
          objectFit: 'contain',
          width: '100%',
          height: '100%',
          display: loading || error ? 'none' : 'block',
        }}
      />
    </Box>
  );
};
