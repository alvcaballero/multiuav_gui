import React, { useRef, useState, useEffect } from 'react';
import { Billboard, Text, Circle, Box, Line } from '@react-three/drei';

const NumberedSphere = ({ position, properties, hideLabel = false }) => {
  return (
    <group position={position} scale={1}>
      {!hideLabel && (
        <Word key={properties.id} position={[0, 0, 0]} color={properties.color ?? '#ff0000'}>
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
            color={properties.color ?? '#ff0000'}
          />
          <Box args={[0.2, 0.2, 0.2]} position={[0, 0, -1]}>
            <meshStandardMaterial color={properties.color ?? '#ff0000'} />
          </Box>
          {properties.gimbal_pitch != null && (
            <group rotation={[(properties.gimbal_pitch * Math.PI) / 180, 0, 0]}>
              <Line
                points={[
                  [0, 0, 0],
                  [0, 0, -1],
                ]}
                color={'#04ff00'}
              />
              <Box args={[0.1, 0.1, 0.1]} position={[0, 0, -1]} rotation={[0, 0, 0]}>
                <meshStandardMaterial color={'#04ff00'} />
              </Box>
            </group>
          )}
        </group>
      )}
    </group>
  );
};

function Word({ children, position, color }) {
  const fontProps = {
    font: '/Inter-Bold.woff',
    fontSize: 0.5,
    letterSpacing: -0.05,
    lineHeight: 1,
    'material-toneMapped': false,
  };
  const ref = useRef();
  const [hovered, setHovered] = useState(false);
  const over = (e) => (e.stopPropagation(), setHovered(true));
  const out = () => setHovered(false);

  useEffect(() => {
    document.body.style.cursor = hovered ? 'pointer' : 'auto';
    return () => (document.body.style.cursor = 'auto');
  }, [hovered]);

  // Only update color imperatively when hover state changes, not every frame.
  useEffect(() => {
    if (ref.current) ref.current.material.color.set(hovered ? 'black' : 'white');
  }, [hovered]);

  return (
    <Billboard position={position}>
      <Circle args={[0.5, 25]}>
        <meshBasicMaterial attach="material" color={color} />
      </Circle>
      <Text
        ref={ref}
        position={[0, 0, 0.1]}
        onPointerOver={over}
        onPointerOut={out}
        onClick={() => console.log('clicked')}
        {...fontProps}
        children={children}
      />
    </Billboard>
  );
}

export default NumberedSphere;
