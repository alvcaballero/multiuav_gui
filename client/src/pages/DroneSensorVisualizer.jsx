import React, { useRef, useEffect, useState } from 'react';
import PropTypes from 'prop-types';
import DistanceSensor from './DistanceSensor';
import { mapIconKey, mapIcons } from '../map/core/preloadImages';
import LinearGauge from './LinearGauge';

const getSensorStyle = (direction, sensorWidth, sensorHeight, containerSize) => {
  const center = containerSize / 2;

  const coneStyle = { position: 'absolute', zIndex: 1 };
  const textStyle = {
    position: 'absolute',
    fontSize: '0.75em',
    color: '#333',
    fontWeight: 'bold',
    zIndex: 2,
    pointerEvents: 'none',
  };

  switch (direction) {
    case 'front':
      coneStyle.top = 0;
      coneStyle.left = center;
      coneStyle.transform = 'translate(-50%, 0) rotate(180deg)';
      textStyle.top = sensorHeight * 0.4;
      textStyle.left = center;
      textStyle.transform = 'translateX(-50%)';
      break;
    case 'back':
      coneStyle.bottom = 0;
      coneStyle.left = center;
      coneStyle.transform = 'translate(-50%, 0)';
      textStyle.bottom = sensorHeight * 0.4;
      textStyle.left = center;
      textStyle.transform = 'translateX(-50%)';
      break;
    case 'left':
      coneStyle.top = center;
      coneStyle.left = 0;
      coneStyle.transform = 'translate(0, -50%) rotateZ(180deg)';
      textStyle.top = center;
      textStyle.left = sensorWidth * 0.3;
      textStyle.transform = 'translateY(-50%)';
      break;
    case 'right':
      coneStyle.top = center;
      coneStyle.right = 0;
      coneStyle.transform = 'translate(0, -50%)';
      textStyle.top = center;
      textStyle.right = sensorWidth * 0.3;
      textStyle.transform = 'translateY(-50%)';
      break;
    default:
      break;
  }

  return { coneStyle, textStyle };
};

/**
 * Ocupa todo el espacio que le da el padre (width + height 100%).
 * El ResizeObserver mide AMBAS dimensiones para que nada se corte.
 */
const DroneSensorVisualizer = ({
  sensorData = { down: 10, front: 10, left: 10, back: 10, right: 10, up: 10 },
  sensorConfig = {
    down: [0, 15],
    front: [0, 15],
    left: [0, 12],
    back: [0, 12],
    right: [0, 12],
    up: [0, 20],
  },
  altitude = 10,
  altitudeASL,
}) => {
  const containerRef = useRef(null);
  const [size, setSize] = useState({ w: 0, h: 0 });

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const ro = new ResizeObserver(([entry]) => {
      setSize({ w: entry.contentRect.width, h: entry.contentRect.height });
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const { w, h } = size;
  const ready = w > 10 && h > 10;

  // El panel top-view es cuadrado con lado = min(h, 65% del ancho)
  const topViewSize = ready ? Math.min(h, Math.round(w * 0.65)) : 0;
  // El gauge toma el resto del ancho
  const gaugeW = ready ? w - topViewSize - 8 : 0;

  // Conos: proporcionales al panel cuadrado
  const sensorConeLen = Math.round(topViewSize * 0.38);
  const sensorConeWide = Math.round(topViewSize * 0.52);
  const droneSize = Math.round(topViewSize * 0.22);

  const { coneStyle: styleFrontCone, textStyle: styleFrontText } = getSensorStyle(
    'front',
    sensorConeWide,
    sensorConeLen,
    topViewSize,
  );
  const { coneStyle: styleBackCone, textStyle: styleBackText } = getSensorStyle(
    'back',
    sensorConeWide,
    sensorConeLen,
    topViewSize,
  );
  const { coneStyle: styleLeftCone, textStyle: styleLeftText } = getSensorStyle(
    'left',
    sensorConeLen,
    sensorConeWide,
    topViewSize,
  );
  const { coneStyle: styleRightCone, textStyle: styleRightText } = getSensorStyle(
    'right',
    sensorConeLen,
    sensorConeWide,
    topViewSize,
  );

  return (
    // Wrapper: ocupa TODO el espacio del padre, sin overflow propio
    <div
      ref={containerRef}
      style={{
        width: '100%',
        height: '100%',
        display: 'flex',
        alignItems: 'center',
        gap: 8,
        overflow: 'hidden',
      }}
    >
      {ready && (
        <>
          {/* ── Vista superior con conos ── */}
          <div
            style={{
              width: topViewSize,
              height: topViewSize,
              position: 'relative',
              flexShrink: 0,
            }}
          >
            <img
              src={mapIcons[mapIconKey('ArrowMap')]}
              alt=""
              style={{
                width: droneSize,
                position: 'absolute',
                top: '50%',
                left: '50%',
                transform: 'translate(-50%,-50%)',
                zIndex: 2,
              }}
            />

            <div style={styleFrontText}>{sensorData.front.toFixed(0)}m</div>
            <div style={styleFrontCone}>
              <DistanceSensor
                distance={sensorData.front}
                limits={sensorConfig.front}
                width={sensorConeWide}
                height={sensorConeLen}
              />
            </div>

            <div style={styleBackText}>{sensorData.back.toFixed(0)}m</div>
            <div style={styleBackCone}>
              <DistanceSensor
                distance={sensorData.back}
                limits={sensorConfig.back}
                width={sensorConeWide}
                height={sensorConeLen}
              />
            </div>

            <div style={styleLeftText}>{sensorData.left.toFixed(0)}m</div>
            <div style={styleLeftCone}>
              <DistanceSensor
                distance={sensorData.left}
                limits={sensorConfig.left}
                width={sensorConeLen}
                height={sensorConeWide}
                orientation="h"
              />
            </div>

            <div style={styleRightText}>{sensorData.right.toFixed(0)}m</div>
            <div style={styleRightCone}>
              <DistanceSensor
                distance={sensorData.right}
                limits={sensorConfig.right}
                width={sensorConeLen}
                height={sensorConeWide}
                orientation="h"
              />
            </div>
          </div>

          {/* ── LinearGauge: ocupa el ancho restante, alto = topViewSize ── */}
          <div style={{ width: gaugeW, height: topViewSize, flexShrink: 0, overflow: 'visible' }}>
            <LinearGauge
              value={altitude}
              valueASL={altitudeASL}
              sensorValue={[sensorData.up, sensorData.down]}
              sensorLimits={{ up: sensorConfig.up, down: sensorConfig.down }}
            />
          </div>
        </>
      )}
    </div>
  );
};

DroneSensorVisualizer.propTypes = {
  sensorData: PropTypes.shape({
    front: PropTypes.number.isRequired,
    back: PropTypes.number.isRequired,
    left: PropTypes.number.isRequired,
    right: PropTypes.number.isRequired,
    up: PropTypes.number.isRequired,
    down: PropTypes.number.isRequired,
  }).isRequired,
};

export default DroneSensorVisualizer;
