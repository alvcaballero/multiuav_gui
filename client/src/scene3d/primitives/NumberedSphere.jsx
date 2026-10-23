import { useRef, useEffect } from 'react';
import { Billboard, Text, Circle, Box, Line } from '@react-three/drei';
import { NO_ROUTE_COLOR } from '../../common/util/styles/routeColors';

// The gimbal direction. The 2D map does not draw it (it is only text in the waypoint popup), so
// there is no map colour to match.
const GIMBAL_COLOR = '#04ff00';

const WORD_FONT_PROPS = {
  font: '/Inter-Bold.woff',
  fontSize: 1.8,
  letterSpacing: -0.05,
  lineHeight: 1,
  'material-toneMapped': false,
};

// A waypoint, as the 2D map draws it (MapMissions): its route's colour (routeColors.js), set by
// R3FMission, and NO_ROUTE_COLOR when it has none — never a colour of its own.
const NumberedSphere = ({ position, properties, hideLabel = false }) => {
  const color = properties.color ?? NO_ROUTE_COLOR;
  return (
    <group position={position} scale={1}>
      {!hideLabel && (
        <Word key={properties.id} position={[0, 0, 0]} color={color}>
          {properties.id ?? 0}
        </Word>
      )}
      {properties.yaw != null && (
        <group rotation={[0, (-properties.yaw * Math.PI) / 180, 0]}>
          <Line
            points={[
              [0, 0, 0],
              [0, 0, -1],
            ]}
            color={color}
          />
          <Box args={[0.2, 0.2, 0.2]} position={[0, 0, -1]}>
            <meshStandardMaterial color={color} />
          </Box>
          {properties.gimbal_pitch != null && (
            <group rotation={[(properties.gimbal_pitch * Math.PI) / 180, 0, 0]}>
              <Line
                points={[
                  [0, 0, 0],
                  [0, 0, -1],
                ]}
                color={GIMBAL_COLOR}
              />
              <Box args={[0.1, 0.1, 0.1]} position={[0, 0, -1]} rotation={[0, 0, 0]}>
                <meshStandardMaterial color={GIMBAL_COLOR} />
              </Box>
            </group>
          )}
        </group>
      )}
    </group>
  );
};

function Word({ children, position, color }) {
  const ref = useRef();
  const hoveredRef = useRef(false);

  const setHovered = (hovered) => {
    hoveredRef.current = hovered;
    document.body.style.cursor = hovered ? 'pointer' : 'auto';
    if (ref.current) ref.current.material.color.set(hovered ? 'black' : 'white');
  };

  const over = (e) => (e.stopPropagation(), setHovered(true));
  const out = () => setHovered(false);

  useEffect(() => {
    return () => {
      if (hoveredRef.current) document.body.style.cursor = 'auto';
    };
  }, []);

  return (
    <Billboard position={position}>
      <Circle args={[1.5, 25]}>
        <meshBasicMaterial attach="material" color={color} />
      </Circle>
      <Text
        ref={ref}
        position={[0, 0, 0.1]}
        onPointerOver={over}
        onPointerOut={out}
        onClick={() => console.log('clicked')}
        {...WORD_FONT_PROPS}
      >
        {children}
      </Text>
    </Billboard>
  );
}

export default NumberedSphere;
