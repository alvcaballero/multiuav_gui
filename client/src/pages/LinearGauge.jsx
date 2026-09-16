import React, { useState, useEffect, useRef } from 'react';
import './LinearGauge.css';
import { mapIconKey, frontIcons } from '../map/core/preloadImages';

/**
 * LinearGauge — altímetro vertical con sensores up/down.
 * Ocupa todo el ancho/alto que le da el padre (width/height 100%).
 * El indicador del valor actual se muestra a la IZQUIERDA de la regla
 * para que nunca se salga por la derecha.
 */
const sensorColor = (val, limits) => {
  const lo = limits[0] + 0.3 * (limits[1] - limits[0]);
  const hi = limits[0] + 0.75 * (limits[1] - limits[0]);
  if (val <= lo) return 'red';
  if (val <= hi) return '#f0b400';
  return '#bbb';
};

const LinearGauge = ({
  value,
  valueASL,
  sensorValue = [7, 5],
  sensorLimits = { up: [0, 20], down: [0, 20] },
  range = 50,
  tickInterval = 5,
  majorTickInterval = 25,
  unit = 'm',
  indicatorColor = '#333',
  tickColor = '#999',
  labelColor = '#333',
}) => {
  const wrapperRef = useRef(null);
  const [size, setSize] = useState({ w: 0, h: 0 });

  useEffect(() => {
    const el = wrapperRef.current;
    if (!el) return;
    const ro = new ResizeObserver(([entry]) => {
      setSize({ w: entry.contentRect.width, h: entry.contentRect.height });
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const { w, h } = size;

  // ── Layout ──────────────────────────────────────────────────────────────
  const SENSOR_W = Math.max(8, Math.round(w * 0.06));
  const DRONE_W = Math.max(20, Math.round(h * 0.12));
  const RULER_X = SENSOR_W + DRONE_W;
  const TICK_MAJ = Math.max(8, Math.round(w * 0.1));
  const TICK_MIN = Math.max(5, Math.round(w * 0.07));
  const FONT = Math.max(9, Math.round(h / 24));
  // Padding vertical para que ticks/labels de los extremos no se corten
  const PAD_Y = Math.max(12, FONT + 2);

  // ── Helpers ─────────────────────────────────────────────────────────────
  const currentMin = value - range / 2;
  // yOfVal mapea sobre [PAD_Y, h - PAD_Y] para que los extremos tengan margen
  const yOfVal = (val) => PAD_Y + (h - 2 * PAD_Y) * (1 - (val - currentMin) / range);

  // ── Ticks ───────────────────────────────────────────────────────────────
  const ticks = [];
  const labels = [];
  if (w > 0 && h > 0) {
    const start = Math.floor(currentMin / tickInterval) * tickInterval;
    const end = Math.ceil((value + range / 2) / tickInterval) * tickInterval;
    for (let i = start; i <= end; i += tickInterval) {
      const y = yOfVal(i);
      if (y < -4 || y > h + 4) continue;
      const major = Math.abs(i % majorTickInterval) < tickInterval / 2;
      ticks.push(
        <div
          key={`t${i}`}
          style={{
            position: 'absolute',
            top: y,
            left: 0,
            width: major ? TICK_MAJ : TICK_MIN,
            height: 2,
            backgroundColor: tickColor,
            transform: 'translateY(-50%)',
          }}
        />,
      );
      if (major) {
        labels.push(
          <div
            key={`l${i}`}
            className="gauge-tick-label"
            style={{
              top: y,
              left: TICK_MAJ + 3,
              fontSize: FONT,
              color: labelColor,
              transform: 'translateY(-50%)',
            }}
          >
            {i}
          </div>,
        );
      }
    }
  }

  // ── Sensor bars ─────────────────────────────────────────────────────────
  const s1 = Math.max(sensorLimits.up[0], Math.min(sensorLimits.up[1], sensorValue[0]));
  const s2 = Math.max(sensorLimits.down[0], Math.min(sensorLimits.down[1], sensorValue[1]));
  // Las barras ocupan la mitad del área útil (entre PAD_Y y h/2, y entre h/2 y h-PAD_Y)
  const usable = (h - 2 * PAD_Y) / 2;
  const upH =
    PAD_Y + usable * (1 - (s1 - sensorLimits.up[0]) / (sensorLimits.up[1] - sensorLimits.up[0]));
  const downH =
    PAD_Y +
    usable * (1 - (s2 - sensorLimits.down[0]) / (sensorLimits.down[1] - sensorLimits.down[0]));
  const upCol = sensorColor(s1, sensorLimits.up);
  const downCol = sensorColor(s2, sensorLimits.down);

  const indicatorY = w > 0 ? yOfVal(value) : h / 2;

  return (
    <div
      ref={wrapperRef}
      style={{ position: 'relative', width: '100%', height: '100%', overflow: 'visible' }}
    >
      {w > 0 && h > 0 && (
        <>
          {/* ── Área de clip (regla + sensores) ──────────────────── */}
          <div style={{ position: 'absolute', inset: 0, overflow: 'hidden' }}>
            {/* Sensor UP barra — de PAD_Y hasta upH */}
            <div
              style={{
                position: 'absolute',
                top: PAD_Y,
                left: 0,
                width: SENSOR_W,
                height: Math.max(0, upH - PAD_Y),
                backgroundColor: upCol,
              }}
            />
            {/* Línea sensor UP */}
            <div
              style={{
                position: 'absolute',
                top: upH,
                left: 0,
                width: SENSOR_W + 6,
                height: 3,
                backgroundColor: upCol === '#bbb' ? tickColor : upCol,
                transform: 'translateY(-50%)',
              }}
            />

            {/* Sensor DOWN barra — de h-PAD_Y hasta h-PAD_Y-downH */}
            <div
              style={{
                position: 'absolute',
                bottom: PAD_Y,
                left: 0,
                width: SENSOR_W,
                height: Math.max(0, downH - PAD_Y),
                backgroundColor: downCol,
              }}
            />
            {/* Línea sensor DOWN */}
            <div
              style={{
                position: 'absolute',
                bottom: downH,
                left: 0,
                width: SENSOR_W + 6,
                height: 3,
                backgroundColor: downCol === '#bbb' ? tickColor : downCol,
                transform: 'translateY(50%)',
              }}
            />

            {/* Línea vertical de la regla */}
            <div
              style={{
                position: 'absolute',
                top: PAD_Y,
                bottom: PAD_Y,
                left: RULER_X,
                width: 2,
                backgroundColor: tickColor,
              }}
            />

            {/* Ticks + labels */}
            <div
              style={{ position: 'absolute', top: 0, left: RULER_X + 2, right: 0, height: '100%' }}
            >
              {ticks}
              {labels}
            </div>

            {/* ASL — esquina inferior derecha */}
            {valueASL != null && (
              <div
                style={{
                  position: 'absolute',
                  bottom: 4,
                  right: 4,
                  fontSize: FONT - 1,
                  color: labelColor,
                  opacity: 0.65,
                  whiteSpace: 'nowrap',
                }}
              >
                {valueASL.toFixed(1)} ASL
              </div>
            )}
          </div>

          {/* Labels sensor fuera del clip */}
          <div
            className="gauge-sensor-label"
            style={{
              top: Math.max(2, upH - FONT - 2),
              fontSize: FONT,
              color: labelColor,
            }}
          >
            {sensorValue[0].toFixed(0)}↑
          </div>
          <div
            className="gauge-sensor-label"
            style={{
              bottom: Math.max(2, downH - FONT - 2),
              fontSize: FONT,
              color: labelColor,
            }}
          >
            {sensorValue[1].toFixed(0)}↓
          </div>

          {/* ── Indicador: ícono + flecha + caja — todos centrados en indicatorY ── */}
          <div
            className="gauge-indicator"
            style={{ top: indicatorY, left: SENSOR_W, transform: 'translateY(-50%)' }}
          >
            {/* Ícono dron vista frontal */}
            <img
              src={frontIcons[mapIconKey('ArrowMap')]}
              alt=""
              style={{ width: DRONE_W, display: 'block', flexShrink: 0 }}
            />
            {/* Punta de flecha → hacia la derecha */}
            <div
              style={{
                width: 0,
                height: 0,
                flexShrink: 0,
                borderTop: '7px solid transparent',
                borderBottom: '7px solid transparent',
                borderLeft: `10px solid ${indicatorColor}`,
              }}
            />
            {/* Caja del valor */}
            <div
              className="gauge-indicator-value"
              style={{
                border: `1px solid ${indicatorColor}`,
                fontSize: FONT + 1,
                color: indicatorColor,
              }}
            >
              {value.toFixed(1)}&thinsp;{unit}
            </div>
          </div>
        </>
      )}
    </div>
  );
};

export default LinearGauge;
