import React from 'react';
import { Billboard, Text ,Circle} from '@react-three/drei';
import * as THREE from 'three';
import {  useFrame } from '@react-three/fiber'
import { useRef, useState, useMemo, useEffect, Suspense } from 'react'


const NumberedSphere = ({ position, number, color = 'red', scale = 1 }) => {
  return (
    <group position={position} scale={scale}>

      <Word key={number} position={[0, 1, 0]}>
        {number}
      </Word>
    </group>
  );
};

function Word({ children, ...props }) {
    const color = new THREE.Color()
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
      ref.current.material.color.lerp(color.set(hovered ? '#fa2720' : 'white'), 0.1)
    })
    return (
      <Billboard {...props}>
        <Circle args={[1, 150]}>
            <meshBasicMaterial attach="material" color="#ff0000" />
        </Circle>
        <Text ref={ref} onPointerOver={over} onPointerOut={out} onClick={() => console.log('clicked')} {...fontProps} children={children} />
      </Billboard>
    )
  }

export default NumberedSphere; 