import { useRef, useEffect, useMemo } from 'react'
import { Canvas, useFrame, useLoader } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera, Environment, Grid, Float, ContactShadows, Center } from '@react-three/drei'
import * as THREE from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import ConnectionError from '../shared/ConnectionError'

function StlModel({ url }: { url: string }) {
  const geom = useLoader(STLLoader, url)
  const meshRef = useRef<THREE.Mesh>(null!)

  return (
    <Float speed={2} rotationIntensity={0.5} floatIntensity={0.5}>
      <Center>
        <mesh ref={meshRef} geometry={geom} castShadow receiveShadow>
          <meshStandardMaterial color="#3b82f6" roughness={0.3} metalness={0.8} />
        </mesh>
      </Center>
    </Float>
  )
}

interface WireProps {
  waypoints: [number, number, number][];
  color?: string;
  radius?: number;
}

function Wire({ waypoints, color = "#ef4444", radius = 0.05 }: WireProps) {
  const curve = useMemo(() => {
    const points = waypoints.map(wp => new THREE.Vector3(...wp));
    return new THREE.CatmullRomCurve3(points);
  }, [waypoints]);

  return (
    <mesh castShadow>
      <tubeGeometry args={[curve, waypoints.length * 4, radius, 8, false]} />
      <meshStandardMaterial color={color} roughness={0.5} />
    </mesh>
  );
}

function PlaceholderModel() {
  const meshRef = useRef<THREE.Mesh>(null!)
  
  useFrame((state) => {
    const t = state.clock.getElapsedTime()
    meshRef.current.rotation.x = Math.cos(t / 4) / 8
    meshRef.current.rotation.y = Math.sin(t / 4) / 8
    meshRef.current.position.y = (1 + Math.sin(t / 1.5)) / 10
  })

  return (
    <Float speed={2} rotationIntensity={0.5} floatIntensity={0.5}>
      <mesh ref={meshRef} castShadow receiveShadow>
        <boxGeometry args={[1, 1, 1]} />
        <meshStandardMaterial color="#3b82f6" roughness={0.3} metalness={0.8} />
      </mesh>
    </Float>
  )
}

interface WireRoute {
  wire_id: string;
  waypoints: [number, number, number][];
  gauge_awg?: number;
}

interface ModelViewerProps {
  className?: string;
  assetUrl?: string | null;
  wireRoutes?: WireRoute[];
  isConnected?: boolean;
  resetTrigger?: number;
}

export default function ModelViewer({ 
  className, 
  assetUrl, 
  wireRoutes = [], 
  isConnected = true, 
  resetTrigger = 0 
}: ModelViewerProps) {
  const controlsRef = useRef<any>(null)

  useEffect(() => {
    if (resetTrigger > 0 && controlsRef.current) {
      const controls = controlsRef.current
      controls.object.position.set(3, 3, 3)
      controls.target.set(0, 0, 0)
      controls.update()
    }
  }, [resetTrigger])

  return (
    <div className={`${className} relative`}>
      {!isConnected && <ConnectionError className="absolute inset-0 z-50" />}
      <Canvas shadows dpr={[1, 2]}>
        <PerspectiveCamera makeDefault position={[3, 3, 3]} fov={50} />
        <OrbitControls 
          ref={controlsRef}
          enableDamping 
          dampingFactor={0.2}
          minDistance={2}
          maxDistance={10}
          makeDefault
        />
        
        <Environment preset="city" />
        <ambientLight intensity={0.5} />
        <spotLight position={[10, 10, 10]} angle={0.15} penumbra={1} intensity={1} castShadow />
        <pointLight position={[-10, -10, -10]} intensity={0.5} />
        
        {assetUrl ? (
            <StlModel url={assetUrl} />
        ) : (
            <PlaceholderModel />
        )}

        {/* Render physical wires */}
        {wireRoutes.map(route => (
          <Wire 
            key={route.wire_id} 
            waypoints={route.waypoints} 
            radius={route.gauge_awg ? 0.05 * (20 / route.gauge_awg) : 0.05}
          />
        ))}
        
        <Grid 
          infiniteGrid 
          fadeDistance={50} 
          fadeStrength={5} 
          cellSize={1} 
          sectionSize={5} 
          sectionThickness={1.5}
          sectionColor="#3b82f6"
          cellColor="#6b7280"
        />
        
        <ContactShadows 
          position={[0, -0.5, 0]} 
          opacity={0.4} 
          scale={10} 
          blur={2} 
          far={4.5} 
        />
      </Canvas>
    </div>
  )
}
