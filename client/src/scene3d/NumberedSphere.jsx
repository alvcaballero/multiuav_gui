import React from 'react';
import { Billboard, Text, Circle, calcPosFromAngles } from '@react-three/drei';
import * as THREE from 'three';
import { useFrame } from '@react-three/fiber'
import { useRef, useState, useMemo, useEffect, Suspense } from 'react'
import { ColorType } from 'maplibre-gl';


const NumberedSphere = ({ position, number, color = '#ff0000', scale = 1 }) => {
  return (
    <group position={position} scale={scale} >

      <Word key={number} position={[0, 0, 0]} color={color}>
        {number}
      </Word>
    </group>
  );
};

function Word({ children, position, color }) {
  const myColor = new THREE.Color()
  const fontProps = { font: '/Inter-Bold.woff', fontSize: 2.5, letterSpacing: -0.05, lineHeight: 1, 'material-toneMapped': false }
  const ref = useRef()
  const [hovered, setHovered] = useState(false)
  const over = (e) => (e.stopPropagation(), setHovered(true))
  const out = () => setHovered(false)
  // Change the mouse cursor on hover¨
  useEffect(() => {
    if (hovered) document.body.style.cursor = 'pointer'
    return () => (document.body.style.cursor = 'auto')
  }, [hovered])
  // Tie component to the render-loop
  useFrame(({ camera }) => {
    ref.current.material.color.lerp(myColor.set(hovered ? '#fa2720' : 'white'), 0.1)
  })
  return (
    <Billboard position={position}>
      <Circle args={[1.8, 150]}>
        <meshBasicMaterial attach="material" color={color} />
      </Circle>
      <Text ref={ref} onPointerOver={over} onPointerOut={out} onClick={() => console.log('clicked')} {...fontProps} children={children} />
    </Billboard>
  )
}

export default NumberedSphere; 