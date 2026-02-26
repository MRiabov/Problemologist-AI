import { useRef, useEffect, useMemo, useState } from 'react'
import { Canvas, useFrame, useLoader } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera, Environment, Grid, Float, ContactShadows, Center, Html } from '@react-three/drei'
import * as THREE from 'three'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'
import { 
  Layers, 
  Play, 
  Pause, 
  RotateCcw, 
  FastForward, 
  Rewind,
  MousePointer2,
  Box,
  Component,
  Zap
} from "lucide-react"
import { Button } from '../ui/button'
import { Badge } from '../ui/badge'
import { useEpisodes } from '../../context/EpisodeContext'
import ConnectionError from '../shared/ConnectionError'
import { cn } from '../../lib/utils'
import { ModelBrowser, type TopologyNode } from './ModelBrowser'

export type SelectionMode = 'FACE' | 'PART' | 'SUBASSEMBLY'

function GlbModel({ url, hiddenParts = [], selectionMode = 'PART', isElectronicsView = false, onSelect, onStructureParsed }: { 
    url: string, 
    hiddenParts: string[], 
    selectionMode?: SelectionMode,
    isElectronicsView?: boolean,
    onSelect?: (partName: string, level: SelectionMode, metadata?: any) => void,
    onStructureParsed?: (nodes: TopologyNode[]) => void
}) {
  const gltf = useLoader(GLTFLoader, url)
  const meshRef = useRef<THREE.Group>(null!)
  const [hovered, setHovered] = useState<string | null>(null)
  const [selected, setSelected] = useState<string | null>(null)

  useEffect(() => {
    if (gltf.scene && onStructureParsed) {
        const collectTopology = (object: THREE.Object3D): TopologyNode | null => {
            // YACV / Occ naming convention support
            let type: TopologyNode['type'] = 'group';
            if (object instanceof THREE.Mesh) type = 'part';
            
            const name = object.name || '';
            if (name.startsWith('face_')) type = 'face';
            else if (name.startsWith('edge_')) type = 'edge';
            else if (name.startsWith('vertex_')) type = 'vertex';
            else if (object.children.length > 0) type = 'assembly';

            if (!name && type === 'group') return null;
            
            const node: TopologyNode = {
                id: object.uuid,
                name: name || (type === 'part' ? 'Part' : 'Assembly'),
                type: type,
                children: []
            };

            object.children.forEach(child => {
                const childNode = collectTopology(child);
                if (childNode) node.children?.push(childNode);
            });

            return node;
        };

        const rootNode = collectTopology(gltf.scene);
        if (rootNode) onStructureParsed([rootNode]);
    }
  }, [gltf.scene, onStructureParsed]);

  useEffect(() => {
    gltf.scene.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        // Handle transparency for electronics view
        if (isElectronicsView) {
            child.material.transparent = true;
            child.material.opacity = 0.2;
        } else {
            if (child.userData.originalMaterial) {
                child.material.transparent = child.userData.originalMaterial.transparent;
                child.material.opacity = child.userData.originalMaterial.opacity;
            } else {
                child.material.transparent = false;
                child.material.opacity = 1.0;
            }
        }

        // Check if the part itself is hidden, or if any of its parents are hidden
        let isParentHidden = false;
        let p: THREE.Object3D | null = child;
        while (p) {
            if (hiddenParts.includes(p.uuid) || (p.name && hiddenParts.includes(p.name))) {
                isParentHidden = true;
                break;
            }
            p = p.parent;
        }
        child.visible = !isParentHidden;

        // Apply Highlighting
        if (child.visible && !isElectronicsView) {
            const target = findTarget(child);
            const isSelected = selected === target.id;
            const isHovered = hovered === target.id;

            if (isSelected) {
                child.material = new THREE.MeshStandardMaterial({ 
                    color: '#3b82f6', 
                    emissive: '#3b82f6', 
                    emissiveIntensity: 0.5,
                    transparent: true,
                    opacity: 0.8
                });
            } else if (isHovered) {
                child.material = new THREE.MeshStandardMaterial({ 
                    color: '#60a5fa', 
                    emissive: '#60a5fa', 
                    emissiveIntensity: 0.2,
                    transparent: true,
                    opacity: 0.9
                });
            } else {
                // Restore original material
                if (child.userData.originalMaterial) {
                    child.material = child.userData.originalMaterial;
                }
            }
        }
      }
    });
  }, [gltf, hiddenParts, selected, hovered, selectionMode, isElectronicsView]);

  // Initial capture of materials
  useEffect(() => {
    gltf.scene.traverse((child) => {
        if (child instanceof THREE.Mesh && !child.userData.originalMaterial) {
            child.userData.originalMaterial = child.material.clone();
        }
    });
  }, [gltf]);

  // Handle Selection Traversal
  const findTarget = (mesh: THREE.Mesh): { object: THREE.Object3D, id: string, type: SelectionMode } => {
    if (selectionMode === 'FACE') {
        return { object: mesh, id: mesh.name || mesh.uuid, type: 'FACE' };
    }

    if (selectionMode === 'PART') {
        let p: THREE.Object3D = mesh;
        while (p.parent && p.parent !== gltf.scene && !p.name.includes('part')) {
            p = p.parent;
        }
        return { object: p, id: p.name || p.uuid, type: 'PART' };
    }

    if (selectionMode === 'SUBASSEMBLY') {
        let p: THREE.Object3D = mesh;
        // Traverse up to find top-level group or group with children
        while (p.parent && p.parent !== gltf.scene) {
            p = p.parent;
        }
        return { object: p, id: p.name || p.uuid, type: 'SUBASSEMBLY' };
    }

    return { object: mesh, id: mesh.name || mesh.uuid, type: 'PART' };
  };

  return (
    <Float speed={2} rotationIntensity={0.5} floatIntensity={0.5}>
      <Center>
        <primitive 
          object={gltf.scene} 
          ref={meshRef}
          castShadow 
          receiveShadow 
          onClick={(e: any) => {
            e.stopPropagation();
            if (e.object instanceof THREE.Mesh) {
              const target = findTarget(e.object);
              setSelected(target.id);
              if (onSelect) {
                // Capture intersection data for precise steering
                const center: [number, number, number] = [e.point.x, e.point.y, e.point.z];
                let normal: [number, number, number] | undefined = undefined;
                
                if (e.face) {
                  const worldNormal = e.face.normal.clone();
                  worldNormal.applyQuaternion(e.object.quaternion);
                  normal = [worldNormal.x, worldNormal.y, worldNormal.z];
                }

                onSelect(target.id, target.type, { 
                  uuid: target.object.uuid,
                  center,
                  normal,
                  // If it's a vertex/edge, we might have more specific data in the future
                  // but for now center + normal is a huge improvement
                });
              }
            }
          }}
          onPointerOver={(e: any) => {
            e.stopPropagation();
            if (e.object instanceof THREE.Mesh) {
                const target = findTarget(e.object);
                setHovered(target.id);
            }
          }}
          onPointerOut={() => setHovered(null)}
        />
      </Center>
    </Float>
  )
}

interface WireProps {
  waypoints: [number, number, number][];
  color?: string;
  radius?: number;
  highlighted?: boolean;
}

function Wire({ waypoints, color = "#ef4444", radius = 0.05, highlighted = false }: WireProps) {
  const curve = useMemo(() => {
    const points = waypoints.map(wp => new THREE.Vector3(...wp));
    return new THREE.CatmullRomCurve3(points);
  }, [waypoints]);

  return (
    <mesh castShadow>
      <tubeGeometry args={[curve, waypoints.length * 4, highlighted ? radius * 2 : radius, 8, false]} />
      <meshStandardMaterial 
        color={highlighted ? "#fbbf24" : color} 
        roughness={0.5} 
        emissive={highlighted ? "#fbbf24" : "#000000"}
        emissiveIntensity={highlighted ? 0.5 : 0}
      />
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
  color?: string;
}

interface ModelViewerProps {
  className?: string;
  assetUrl?: string | null;
  assetUrls?: string[];
  wireRoutes?: WireRoute[];
  isConnected?: boolean;
  resetTrigger?: number;
  topologyNodes?: TopologyNode[];
  onTopologyChange?: (nodes: TopologyNode[]) => void;
}

export default function ModelViewer({ 
  className, 
  assetUrl, 
  assetUrls = [],
  wireRoutes = [], 
  isConnected = true, 
  resetTrigger = 0,
  topologyNodes = [],
  onTopologyChange,
  onRebuildModel
}: ModelViewerProps & { onRebuildModel?: () => void }) {
  const controlsRef = useRef<any>(null)
  const { addToContext } = useEpisodes();
  const [hiddenParts, setHiddenParts] = useState<string[]>([])
  const [isPlaying, setIsPlaying] = useState(false)
  const [currentTime, setCurrentTime] = useState(0)
  const [showTopology, setShowTopology] = useState(true) // Default to open for Model Browser
  const [isElectronicsView, setIsElectronicsView] = useState(false)
  const [selectionMode, setSelectionMode] = useState<SelectionMode>('PART')

  const urls = useMemo(() => {
    const all = [...assetUrls];
    if (assetUrl && !all.includes(assetUrl)) {
      all.push(assetUrl);
    }
    return all;
  }, [assetUrl, assetUrls]);

  useEffect(() => {
    if (resetTrigger > 0 && controlsRef.current) {
      const controls = controlsRef.current
      controls.object.position.set(3, 3, 3)
      controls.target.set(0, 0, 0)
      controls.update()
    }
  }, [resetTrigger])

  return (
    <div className={cn(className, "relative group flex overflow-hidden bg-slate-950")}>
      {!isConnected && <ConnectionError className="absolute inset-0 z-50" />}
      
      {/* Model Browser Sidebar (Inventor Style) */}
      <ModelBrowser 
        nodes={topologyNodes}
        hiddenParts={hiddenParts}
        onToggleVisibility={(id) => {
          setHiddenParts(prev => 
              prev.includes(id) ? prev.filter(p => p !== id) : [...prev, id]
          )
        }}
        className={cn(
          "w-72 shrink-0 z-20 transition-all duration-150",
          showTopology ? "opacity-100" : "w-0 opacity-0 overflow-hidden border-r-0 pointer-events-none"
        )}
      />

      {/* Main Viewport Area */}
      <div className="flex-1 flex flex-col relative min-w-0">
      {!showTopology && (
        <Button
          variant="secondary"
          size="icon"
          title="Toggle Model Browser"
          data-testid="model-browser-toggle"
          className="absolute top-4 left-4 z-30 h-8 w-8 rounded-full shadow-lg border-primary/20"
          onClick={() => setShowTopology(true)}
        >
          <Layers className="h-4 w-4" />
        </Button>
      )}

      {/* Main Viewport */}
      <div className="flex-1 min-h-0 relative">
        <Canvas shadows dpr={[1, 2]} data-testid="main-canvas">
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
            
            {urls.length > 0 ? (
                urls.map(url => (
                    <GlbModel 
                        key={url} 
                        url={url} 
                        hiddenParts={hiddenParts} 
                        selectionMode={selectionMode}
                        isElectronicsView={isElectronicsView}
                        onStructureParsed={onTopologyChange}
                        onSelect={(name, level, metadata) => {
                            addToContext({
                                id: `cad-${name}`,
                                type: 'cad',
                                label: name,
                                metadata: { 
                                    part: name,
                                    level: level,
                                    ...metadata
                                }
                            });
                        }}
                    />
                ))
            ) : (
                <PlaceholderModel />
            )}
            
            {/* Rebuild Prompt Overlay */}
            {urls.length === 0 && onRebuildModel && (
                <Html center>
                    <div data-testid="no-model-overlay" className="w-64 text-center pointer-events-auto">
                         <div className="bg-slate-900/90 backdrop-blur border border-slate-700 p-4 rounded-xl shadow-2xl flex flex-col items-center gap-3">
                            <h3 data-testid="no-model-title" className="text-sm font-bold text-slate-200">No Model Loaded</h3>
                            <Button 
                                data-testid="rebuild-model-button"
                                size="sm" 
                                variant="outline" 
                                onClick={onRebuildModel}
                                className="w-full gap-2 border-primary/50 text-primary hover:bg-primary/10 hover:text-primary"
                            >
                                <RotateCcw className="h-3 w-3" />
                                Rebuild Assets
                            </Button>
                         </div>
                    </div>
                </Html>
            )}

            {/* Render physical wires */}
            {wireRoutes.map(route => {
                // Determine wire color based on ID/metadata if available, 
                // fallback to default red
                let color = route.color || "#ef4444";
                if (route.wire_id.toLowerCase().includes("gnd") || route.wire_id.toLowerCase().includes("ground")) {
                    color = "#171717"; // Black
                } else if (route.wire_id.toLowerCase().includes("vcc") || route.wire_id.toLowerCase().includes("power")) {
                    color = "#dc2626"; // Red
                }

                return (
                    <Wire 
                        key={route.wire_id} 
                        waypoints={route.waypoints} 
                        color={color}
                        radius={route.gauge_awg ? 0.05 * (20 / route.gauge_awg) : 0.05}
                        highlighted={isElectronicsView}
                    />
                );
            })}
            
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

        {/* Viewport Buttons */}
        <div className="absolute top-4 left-4 z-10 flex flex-col gap-2">
            <Button 
                variant="secondary" 
                size="icon" 
                title="Toggle Model Browser"
                data-testid="model-browser-toggle"
                className={cn(
                  "h-8 w-8 rounded-full shadow-lg border-primary/20",
                  showTopology ? "bg-primary text-primary-foreground" : "hidden"
                )}
                onClick={() => setShowTopology(!showTopology)}
            >
                <Layers className="h-4 w-4" />
            </Button>
            <Button 
                variant="secondary" 
                size="icon" 
                className={cn("h-8 w-8 rounded-full shadow-lg border-primary/20", isElectronicsView && "bg-yellow-500 text-yellow-950")}
                onClick={() => setIsElectronicsView(!isElectronicsView)}
                title="Toggle Electronics View"
            >
                <Zap className="h-4 w-4" />
            </Button>

            <div className="flex flex-col gap-1 bg-slate-900/80 backdrop-blur border border-slate-700 p-1 rounded-full shadow-xl mt-2">
                <Button 
                    variant="ghost" 
                    size="icon" 
                    title="Face Selection"
                    className={cn("h-7 w-7 rounded-full", selectionMode === 'FACE' && "bg-primary text-primary-foreground")}
                    onClick={() => setSelectionMode('FACE')}
                >
                    <MousePointer2 className="h-3.5 w-3.5" />
                </Button>
                <Button 
                    variant="ghost" 
                    size="icon" 
                    title="Part Selection"
                    className={cn("h-7 w-7 rounded-full", selectionMode === 'PART' && "bg-primary text-primary-foreground")}
                    onClick={() => setSelectionMode('PART')}
                >
                    <Box className="h-3.5 w-3.5" />
                </Button>
                <Button 
                    variant="ghost" 
                    size="icon" 
                    title="Subassembly Selection"
                    className={cn("h-7 w-7 rounded-full", selectionMode === 'SUBASSEMBLY' && "bg-primary text-primary-foreground")}
                    onClick={() => setSelectionMode('SUBASSEMBLY')}
                >
                    <Component className="h-3.5 w-3.5" />
                </Button>
            </div>
        </div>
      </div>

      {/* Simulation Controls Footer */}
      <div className="shrink-0 p-3 bg-card/80 backdrop-blur-md border-t border-border/50 flex flex-col gap-3">
          <div className="flex items-center gap-4">
              <Button 
                variant="ghost" 
                size="icon" 
                data-testid="simulation-play-toggle"
                className="h-8 w-8 text-primary shrink-0"
                onClick={() => setIsPlaying(!isPlaying)}
              >
                  {isPlaying ? <Pause className="h-5 w-5 fill-current" /> : <Play className="h-5 w-5 fill-current" />}
              </Button>
              <div className="flex-1 relative flex items-center">
                  <input 
                      data-testid="simulation-timeline-slider"
                      type="range" 
                      min="0" 
                      max="100" 
                      value={currentTime} 
                      onChange={(e) => setCurrentTime(parseInt(e.target.value))}
                      className="w-full h-1.5 bg-muted rounded-lg appearance-none cursor-pointer accent-primary"
                  />
                  <div 
                    className="absolute top-0 bottom-0 left-0 bg-primary/20 rounded-lg pointer-events-none" 
                    style={{ width: `${currentTime}%` }} 
                  />
              </div>
              <div className="flex items-center gap-1 shrink-0 px-2 font-mono text-[10px] text-muted-foreground w-20 justify-end">
                  <span data-testid="simulation-current-time" className="text-foreground font-bold">{(currentTime / 10).toFixed(1)}s</span>
                  <span>/ 10.0s</span>
              </div>
          </div>
          <div className="flex items-center justify-between px-1">
              <div className="flex gap-2">
                   <Button
                      variant="ghost"
                      size="icon"
                      data-testid="simulation-reset-button"
                      className="h-6 w-6 text-muted-foreground hover:text-foreground"
                      onClick={() => {
                        setCurrentTime(0);
                        setIsPlaying(false);
                      }}
                   >
                      <RotateCcw className="h-3 w-3" />
                   </Button>
                   <Button variant="ghost" size="icon" className="h-6 w-6 text-muted-foreground hover:text-foreground">
                      <Rewind className="h-3 w-3" />
                   </Button>
                   <Button variant="ghost" size="icon" className="h-6 w-6 text-muted-foreground hover:text-foreground">
                      <FastForward className="h-3 w-3" />
                   </Button>
              </div>
              <Badge variant="outline" className="text-[8px] font-black tracking-widest uppercase border-primary/20 bg-primary/5 text-primary">
                  Dynamics Enabled
              </Badge>
          </div>
      </div>
      </div>
    </div>
  )
}
