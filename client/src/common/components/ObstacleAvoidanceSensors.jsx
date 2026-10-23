import PropTypes from 'prop-types';
import { Typography } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import NavigationIcon from '@mui/icons-material/Navigation';

import {
  STOP_DISTANCE,
  CAUTION_DISTANCE,
  obstacleColor,
  obstacleTextColor,
} from '../util/obstacles';
import { HAZARD } from '../theme/palette';

// Top-down view of the collision sensors: the drone in the center (front = up), and per
// horizontal side a ray to the obstacle, a wall at its distance and a label. Up/down
// are bars on the right. Distances in meters; >= RANGE reads as "clear".
//
// Data order on the wire is position.attributes.obstacle_info = [down, front, right, back,
// left, up]; callers map it to { front, back, left, right, up, down } with obstacleFromInfo
// (common/util/obstacles.js).

const SIZE = 160;
const C = SIZE / 2; // center
const DRONE = 11; // half the drone glyph — rays start at its edge
const SPAN = 57; // px the ray grows from 0 m to RANGE
const RANGE = 10; // m — horizontal
const V_RANGE = 15; // m — up/down bars
const WALL = 36;
const RINGS = [26, 52, 78];

const useStyles = makeStyles()((theme) => ({
  root: {
    display: 'flex',
    gap: theme.spacing(1.5),
    alignItems: 'stretch',
    fontVariantNumeric: 'tabular-nums',
  },
  view: {
    position: 'relative',
    width: SIZE,
    height: SIZE,
    flex: 'none',
    overflow: 'hidden',
  },
  ring: {
    position: 'absolute',
    borderRadius: '50%',
    border: `1px dashed ${theme.palette.divider}`,
  },
  drone: {
    position: 'absolute',
    left: C - DRONE,
    top: C - DRONE,
    width: DRONE * 2,
    height: DRONE * 2,
    border: `1px solid ${theme.palette.text.primary}`,
    background: theme.palette.background.paper,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    '& svg': { fontSize: 16 },
  },
  ray: {
    position: 'absolute',
    opacity: 0.35,
    transition: 'all .4s',
  },
  wall: {
    position: 'absolute',
    transition: 'all .4s',
  },
  label: {
    position: 'absolute',
    transform: 'translate(-50%, -50%)',
    fontSize: 11,
    fontWeight: 600,
    padding: '0 3px',
    background: theme.palette.background.paper,
    whiteSpace: 'nowrap',
    transition: 'all .4s',
    pointerEvents: 'none',
  },
  side: {
    flex: 1,
    minWidth: 0,
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(1),
  },
  value: {
    fontSize: 19,
    fontWeight: 600,
    lineHeight: 1.1,
  },
  track: {
    height: 4,
    background: theme.palette.action.hover,
  },
  bar: {
    height: '100%',
    transition: 'width .4s',
  },
  legend: {
    marginTop: 'auto',
    display: 'flex',
    flexDirection: 'column',
    gap: 2,
    fontSize: 11,
    color: theme.palette.text.secondary,
  },
  dot: {
    display: 'inline-block',
    width: 8,
    height: 8,
    marginRight: 6,
  },
}));

// Box geometry of one side's ray, wall and label, in px from the view's top-left.
const geometry = (side, d) => {
  const L = DRONE + (Math.min(d, RANGE) / RANGE) * SPAN;
  const near = C - (L + DRONE) / 2;
  const far = C + (L + DRONE) / 2;
  switch (side) {
    case 'front':
      return {
        ray: { left: C - 1, top: C - L, width: 2, height: L - DRONE },
        wall: { left: C - WALL / 2, top: C - L - 2, width: WALL, height: 3 },
        label: { left: C, top: near },
      };
    case 'back':
      return {
        ray: { left: C - 1, top: C + DRONE, width: 2, height: L - DRONE },
        wall: { left: C - WALL / 2, top: C + L - 1, width: WALL, height: 3 },
        label: { left: C, top: far },
      };
    case 'left':
      return {
        ray: { left: C - L, top: C - 1, width: L - DRONE, height: 2 },
        wall: { left: C - L - 2, top: C - WALL / 2, width: 3, height: WALL },
        label: { left: near, top: C },
      };
    default:
      return {
        ray: { left: C + DRONE, top: C - 1, width: L - DRONE, height: 2 },
        wall: { left: C + L - 1, top: C - WALL / 2, width: 3, height: WALL },
        label: { left: far, top: C },
      };
  }
};

const ObstacleAvoidanceSensors = ({ obstacleData }) => {
  const { classes } = useStyles();

  return (
    <div className={classes.root}>
      <div className={classes.view}>
        {RINGS.map((r) => (
          <div
            key={r}
            className={classes.ring}
            style={{ left: C - r, top: C - r, width: r * 2, height: r * 2 }}
          />
        ))}
        {['front', 'back', 'left', 'right'].map((side) => {
          const d = obstacleData[side];
          const g = geometry(side, d);
          return (
            <div key={side}>
              <div className={classes.ray} style={{ ...g.ray, background: obstacleColor(d) }} />
              <div className={classes.wall} style={{ ...g.wall, background: obstacleColor(d) }} />
              <span className={classes.label} style={{ ...g.label, color: obstacleTextColor(d) }}>
                {d >= RANGE ? `>${RANGE} m` : `${d.toFixed(1)} m`}
              </span>
            </div>
          );
        })}
        <div className={classes.drone} title="Front">
          <NavigationIcon />
        </div>
      </div>
      <div className={classes.side}>
        {[
          ['Up', obstacleData.up],
          ['Down · ground', obstacleData.down],
        ].map(([label, d]) => (
          <div key={label}>
            <Typography
              variant="overline"
              color="textSecondary"
              component="div"
              sx={{ lineHeight: 1.2 }}
            >
              {label}
            </Typography>
            <div className={classes.value} style={{ color: obstacleTextColor(d) }}>
              {d >= V_RANGE ? `>${V_RANGE} m` : `${d.toFixed(1)} m`}
            </div>
            <div className={classes.track}>
              <div
                className={classes.bar}
                style={{
                  width: `${Math.min(100, (d / V_RANGE) * 100)}%`,
                  background: obstacleColor(d),
                }}
              />
            </div>
          </div>
        ))}
        <div className={classes.legend}>
          <span>
            <span className={classes.dot} style={{ background: HAZARD.danger }} />
            {`< ${STOP_DISTANCE} m`}
          </span>
          <span>
            <span className={classes.dot} style={{ background: HAZARD.caution }} />
            {`${STOP_DISTANCE}–${CAUTION_DISTANCE} m`}
          </span>
          <span>
            <span className={classes.dot} style={{ background: HAZARD.clear }} />
            {`> ${CAUTION_DISTANCE} m`}
          </span>
        </div>
      </div>
    </div>
  );
};

ObstacleAvoidanceSensors.propTypes = {
  obstacleData: PropTypes.shape({
    front: PropTypes.number.isRequired,
    back: PropTypes.number.isRequired,
    left: PropTypes.number.isRequired,
    right: PropTypes.number.isRequired,
    up: PropTypes.number.isRequired,
    down: PropTypes.number.isRequired,
  }).isRequired,
};

export default ObstacleAvoidanceSensors;
