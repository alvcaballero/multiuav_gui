// Collision sensor (obstacle_info) helpers shared by the device pages and the sensor widget.
import { HAZARD, HAZARD_TEXT } from '../theme/palette';

export const STOP_DISTANCE = 2; // m — below: stop
export const CAUTION_DISTANCE = 5; // m — below: caution

/** position.attributes.obstacle_info = [down, front, right, back, left, up] → named sides, or null. */
export const obstacleFromInfo = (info) =>
  info
    ? { down: info[0], front: info[1], right: info[2], back: info[3], left: info[4], up: info[5] }
    : null;

/** 'danger' | 'caution' | 'clear' (a HAZARD key) for a distance. */
export const obstacleSeverity = (d) =>
  d < STOP_DISTANCE ? 'danger' : d < CAUTION_DISTANCE ? 'caution' : 'clear';

/** The HAZARD colour for a distance (a fill: a bar, a ray, the alert). */
export const obstacleColor = (d) => HAZARD[obstacleSeverity(d)];
/** The same, for a distance written as text on a light background. */
export const obstacleTextColor = (d) => HAZARD_TEXT[obstacleSeverity(d)];
