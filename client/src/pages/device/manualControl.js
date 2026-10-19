// Manual control by discrete steps: each press adds ±step to a PENDING relative move, which
// is previewed on the map and only sent on Apply. Body frame: fwd/right in meters relative
// to the UAV heading, up in meters, yaw in degrees clockwise; gp/gy = gimbal pitch/yaw.
// The server turns the move into an absolute PX4 target (server/models/vehicleCommand.js);
// relativeTarget below is the same math, used only for the preview.

export const STEPS = [
  { label: '1 m · 5°', m: 1, yaw: 5, gimbal: 5 },
  { label: '5 m · 15°', m: 5, yaw: 15, gimbal: 10 },
  { label: '10 m · 45°', m: 10, yaw: 45, gimbal: 30 },
];

export const EMPTY_PENDING = { fwd: 0, right: 0, up: 0, yaw: 0, gp: 0, gy: 0 };

export const GIMBAL_PITCH_MIN = -90;
export const GIMBAL_PITCH_MAX = 0;

// key → [pending field, sign]
export const KEY_MAP = {
  w: ['fwd', 1],
  s: ['fwd', -1],
  d: ['right', 1],
  a: ['right', -1],
  r: ['up', 1],
  f: ['up', -1],
  e: ['yaw', 1],
  q: ['yaw', -1],
  i: ['gp', 1],
  k: ['gp', -1],
  l: ['gy', 1],
  j: ['gy', -1],
};

const EARTH_RADIUS = 6378137;
const toRad = (deg) => (deg * Math.PI) / 180;
const toDeg = (rad) => (rad * 180) / Math.PI;
const round1 = (v) => Math.round(v * 10) / 10;

export const hasPending = (pending) => Object.values(pending).some((v) => v !== 0);

export const hasPendingMove = ({ fwd, right, up, yaw }) => Boolean(fwd || right || up || yaw);

export const hasPendingGimbal = ({ gp, gy }) => Boolean(gp || gy);

/** Adds `delta` ({field: ±1}) times the step size; keeps the gimbal pitch inside its range. */
export const nudge = (pending, delta, step, gimbalPitch = 0) => {
  const next = { ...pending };
  Object.entries(delta).forEach(([field, sign]) => {
    const size =
      field === 'yaw' ? step.yaw : field === 'gp' || field === 'gy' ? step.gimbal : step.m;
    next[field] = round1(next[field] + sign * size);
  });
  next.gp = Math.max(
    GIMBAL_PITCH_MIN - gimbalPitch,
    Math.min(GIMBAL_PITCH_MAX - gimbalPitch, next.gp),
  );
  return next;
};

/** Where the UAV would end up: {latitude, longitude, altitude, course}. */
export const relativeTarget = (position, { fwd, right, up, yaw }) => {
  const h = toRad(position.course ?? 0);
  const east = fwd * Math.sin(h) + right * Math.cos(h);
  const north = fwd * Math.cos(h) - right * Math.sin(h);
  return {
    latitude: position.latitude + toDeg(north / EARTH_RADIUS),
    longitude:
      position.longitude + toDeg(east / (EARTH_RADIUS * Math.cos(toRad(position.latitude)))),
    altitude: (position.altitude ?? 0) + up,
    course: ((((position.course ?? 0) + yaw) % 360) + 360) % 360,
  };
};

/** Map preview of a pending move ({from, to, course, label} for MapGhostTarget), or null. */
export const ghostTarget = (position, pending) => {
  if (!hasPendingMove(pending) || position?.latitude == null) return null;
  const target = relativeTarget(position, pending);
  const home = position.attributes?.home;
  const altitude = home ? target.altitude - home[2] : target.altitude;
  return {
    from: [position.longitude, position.latitude],
    to: [target.longitude, target.latitude],
    course: target.course,
    label: `Target ${Math.round(altitude)} m`,
  };
};

const signed = (v) => `${v > 0 ? '+' : v < 0 ? '−' : ''}${Math.abs(v)}`;

export const formatSigned = signed;

export const pendingChips = (p) => {
  const chips = [];
  if (p.fwd) chips.push(`${p.fwd > 0 ? 'Fwd' : 'Back'} ${Math.abs(p.fwd)} m`);
  if (p.right) chips.push(`${p.right > 0 ? 'Right' : 'Left'} ${Math.abs(p.right)} m`);
  if (p.up) chips.push(`${p.up > 0 ? 'Up' : 'Down'} ${Math.abs(p.up)} m`);
  if (p.yaw) chips.push(`Yaw ${signed(p.yaw)}°`);
  if (p.gp) chips.push(`Gimbal pitch ${signed(p.gp)}°`);
  if (p.gy) chips.push(`Gimbal yaw ${signed(p.gy)}°`);
  return chips;
};
