import { useRef } from 'react';
import { useGLTF } from '@react-three/drei';
import * as THREE from 'three';

export function Model3D({ path }: { path: string }) {
  const ref = useRef<THREE.Group>();
  const { scene } = useGLTF(path);

  // Set the scene's background to transparent
  scene.background = null;
  
  // Ensure proper material rendering
  scene.traverse((child) => {
    if (child instanceof THREE.Mesh) {
      child.material.needsUpdate = true;
      child.material.side = THREE.DoubleSide;
    }
  });

  return (
    <primitive 
      ref={ref} 
      object={scene} 
      scale={1.5}
      position={[0, -1, 0]}
      rotation={[0, 0, 0]}
    />
  );
}