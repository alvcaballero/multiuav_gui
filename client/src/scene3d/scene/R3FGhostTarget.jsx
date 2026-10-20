import { useSelector } from 'react-redux';
import { useTheme } from '@mui/material';
import { Billboard, Line, Text } from '@react-three/drei';
import { LatLon2XYZ } from '../core/convertion';

const HEADING_TICK_M = 6; // length of the arrow drawn from the target toward its heading
const TARGET_RADIUS_M = 1.5;
const LABEL_OFFSET_M = 4;

/** [lon, lat] + altitude → scene vector, same axes as R3FDevices: x east, y up, z = -north. */
const toScene = (origin, [lng, lat], alt) => {
  const [x, y] = LatLon2XYZ(origin, { lng, lat, alt });
  return [x, alt, -y];
};

/**
 * 3D twin of MapGhostTarget: pending manual move as a dashed path from the UAV to the target,
 * a translucent marker at the target, an arrow toward the target heading, a drop line to the
 * ground (so the altitude change reads in 3D) and a label. Renders nothing while `to` is null.
 * Altitudes use the convention of R3FDevices (above home when known, AMSL otherwise).
 * @param {[number, number]} from [lon, lat] of the UAV
 * @param {[number, number]|null} to [lon, lat] of the target
 * @param {number} fromAltitude
 * @param {number} toAltitude
 * @param {number} course target heading, degrees from north
 * @param {string} label
 */
const R3FGhostTarget = ({ from, to, fromAltitude = 0, toAltitude = 0, course = 0, label = '' }) => {
  const theme = useTheme();
  const origin = useSelector((state) => state.session.scene3d.origin);
  if (!from || !to) return null;

  const color = theme.palette.primary.main;
  const start = toScene(origin, from, fromAltitude);
  const end = toScene(origin, to, toAltitude);
  const heading = (course * Math.PI) / 180;
  const tip = [
    end[0] + HEADING_TICK_M * Math.sin(heading),
    end[1],
    end[2] - HEADING_TICK_M * Math.cos(heading),
  ];

  return (
    <group>
      <Line points={[start, end]} color={color} lineWidth={2} dashed dashSize={1.5} gapSize={1} />
      <Line points={[end, tip]} color={color} lineWidth={3} />
      <Line
        points={[end, [end[0], 0, end[2]]]}
        color={color}
        lineWidth={1}
        dashed
        dashSize={0.5}
        gapSize={0.5}
        transparent
        opacity={0.6}
      />
      <mesh position={end}>
        <sphereGeometry args={[TARGET_RADIUS_M, 24, 16]} />
        <meshBasicMaterial color={color} transparent opacity={0.35} depthWrite={false} />
      </mesh>
      <Billboard position={[end[0], end[1] + LABEL_OFFSET_M, end[2]]}>
        <Text
          fontSize={1.6}
          color={color}
          outlineWidth={0.08}
          outlineColor="#ffffff"
          material-toneMapped={false}
        >
          {label}
        </Text>
      </Billboard>
    </group>
  );
};

export default R3FGhostTarget;
