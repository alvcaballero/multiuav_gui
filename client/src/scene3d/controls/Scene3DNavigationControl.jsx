import { useState, useEffect } from 'react';

const ZoomInIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="29" height="29" fill="#333" viewBox="0 0 29 29">
    <path d="M14.5 8.5c-.75 0-1.5.75-1.5 1.5v3h-3c-.75 0-1.5.75-1.5 1.5S9.25 16 10 16h3v3c0 .75.75 1.5 1.5 1.5S16 19.75 16 19v-3h3c.75 0 1.5-.75 1.5-1.5S19.75 13 19 13h-3v-3c0-.75-.75-1.5-1.5-1.5" />
  </svg>
);

const ZoomOutIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="29" height="29" fill="#333" viewBox="0 0 29 29">
    <path d="M10 13c-.75 0-1.5.75-1.5 1.5S9.25 16 10 16h9c.75 0 1.5-.75 1.5-1.5S19.75 13 19 13z" />
  </svg>
);

const TopViewIcon = () => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    width="29"
    height="29"
    viewBox="0 0 29 29"
    fill="none"
    stroke="#333"
    strokeWidth="1.5"
  >
    <circle cx="14.5" cy="14.5" r="5" fill="none" />
    <circle cx="14.5" cy="14.5" r="2" fill="#333" stroke="none" />
    <line x1="14.5" y1="6" x2="14.5" y2="10" />
    <line x1="14.5" y1="19" x2="14.5" y2="23" />
    <line x1="6" y1="14.5" x2="10" y2="14.5" />
    <line x1="19" y1="14.5" x2="23" y2="14.5" />
  </svg>
);

const CompassIcon = ({ bearing }) => {
  const scale = 1;
  //const scale = pitch !== 0 ? 1 / Math.pow(Math.cos(pitchRad), 0.5) : 1;

  //const transform = `scale(${scale.toFixed(4)}) rotateX(${pitch}deg) rotateZ(${-bearing}deg)`;
  const transform = `scale(${scale.toFixed(4)})  rotateZ(${-bearing}deg)`;

  return (
    <svg
      xmlns="http://www.w3.org/2000/svg"
      width="29"
      height="29"
      viewBox="0 0 29 29"
      style={{ transform, transition: 'transform 0.05s linear', display: 'block' }}
    >
      <path d="m10.5 16 4 8 4-8z" fill="#e74c3c" />
      <path d="m10.5 14 4-8 4 8z" fill="#ccc" />
    </svg>
  );
};

const btnStyle = {
  backgroundColor: 'transparent',
  border: 0,
  boxSizing: 'border-box',
  cursor: 'pointer',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  height: '29px',
  outline: 'none',
  padding: 0,
  width: '29px',
};

const dividerStyle = { borderTop: '1px solid #ddd' };

const containerStyle = {
  position: 'absolute',
  top: '100px',
  right: '20px',
  zIndex: 10,
  background: '#fff',
  borderRadius: '4px',
  boxShadow: '0 0 0 2px rgba(0,0,0,0.1)',
  perspective: '150px',
};

const dispatch = (event) => window.dispatchEvent(new CustomEvent(event));

const Scene3DNavigationControl = () => {
  const [cam, setCam] = useState({ bearing: 0, pitch: 0 });

  useEffect(() => {
    const handler = (e) => setCam({ bearing: e.detail.bearing, pitch: e.detail.pitch });
    window.addEventListener('camera-azimuth', handler);
    return () => window.removeEventListener('camera-azimuth', handler);
  }, []);

  return (
    <div style={containerStyle}>
      <button
        type="button"
        title="Zoom in"
        style={btnStyle}
        onClick={() => dispatch('camera-zoom-in')}
      >
        {' '}
        <ZoomInIcon />{' '}
      </button>
      <div style={dividerStyle}>
        <button
          type="button"
          title="Zoom out"
          style={btnStyle}
          onClick={() => dispatch('camera-zoom-out')}
        >
          {' '}
          <ZoomOutIcon />{' '}
        </button>
      </div>
      <div style={dividerStyle}>
        <button
          type="button"
          title="Reset North"
          style={btnStyle}
          onClick={() => dispatch('camera-orient-north')}
        >
          <CompassIcon roll={cam.roll} bearing={cam.bearing} pitch={cam.pitch} />
        </button>
      </div>
      <div style={dividerStyle}>
        <button
          type="button"
          title="Top view"
          style={btnStyle}
          onClick={() => dispatch('camera-top-view')}
        >
          {' '}
          <TopViewIcon />{' '}
        </button>
      </div>
    </div>
  );
};

export default Scene3DNavigationControl;
