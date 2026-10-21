import { useEffect, useRef, useState } from 'react';

const MAX_LINES = 80;
const POSITION_FIELDS = ['latitude', 'longitude', 'altitude'];
// Bookkeeping fields that change on every message and say nothing by themselves.
const SKIPPED_FIELDS = ['attributes', 'deviceTime', 'serverTime', 'fixTime', 'id'];

const timestamp = () => new Date().toTimeString().slice(0, 8);
const same = (a, b) => JSON.stringify(a) === JSON.stringify(b);

// One line per field that changed between two snapshots of the position. The server
// batches telemetry at 2 Hz, so this is what reached the GCS, not every ROS message.
const diffLines = (prev, next) => {
  if (!next) return [];
  const lines = [];
  if (!prev || POSITION_FIELDS.some((f) => prev[f] !== next[f])) {
    lines.push({
      topic: 'position',
      payload: JSON.stringify(Object.fromEntries(POSITION_FIELDS.map((f) => [f, next[f]]))),
    });
  }
  Object.keys(next)
    .filter((key) => !POSITION_FIELDS.includes(key) && !SKIPPED_FIELDS.includes(key))
    .filter((key) => prev && !same(prev[key], next[key]))
    .forEach((key) => lines.push({ topic: key, payload: JSON.stringify(next[key]) }));
  Object.keys(next.attributes ?? {})
    .filter((key) => !same(prev?.attributes?.[key], next.attributes[key]))
    .forEach((key) => lines.push({ topic: key, payload: JSON.stringify(next.attributes[key]) }));
  return lines;
};

/** Live text stream of a device's telemetry changes plus the commands sent from this page. */
export const useTelemetryLog = (position) => {
  const [lines, setLines] = useState([]);
  const [paused, setPaused] = useState(false);
  // Last snapshot already turned into lines (a ref, not usePrevious: StrictMode re-runs the
  // mount effect and would log the first snapshot twice).
  const lastRef = useRef(null);
  const counterRef = useRef(0);

  const append = (entries) => {
    if (paused || !entries.length) return;
    const time = timestamp();
    const stamped = entries.map((entry) => {
      counterRef.current += 1;
      return { ...entry, time, key: counterRef.current };
    });
    setLines((current) => [...current, ...stamped].slice(-MAX_LINES));
  };

  useEffect(() => {
    if (paused || lastRef.current === position) return;
    append(diffLines(lastRef.current, position));
    lastRef.current = position;
    // `append` only closes over refs, setters and `paused`
    // eslint-disable-next-line @eslint-react/exhaustive-deps
  }, [position, paused]); // oxlint-disable-line react-doctor/exhaustive-deps

  const log = (topic, payload) => append([{ topic, payload }]);

  return { lines, paused, setPaused, log };
};
