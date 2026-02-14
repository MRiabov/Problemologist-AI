import React, { useMemo } from 'react';
import { cn } from "../../lib/utils";
import { Badge } from "../ui/badge";

interface Component {
  component_id: string;
  type: string;
}

interface Wire {
  wire_id: string;
  from: { component: string; terminal: string };
  to: { component: string; terminal: string };
}

interface ElectronicsSection {
  power_supply: { voltage_dc: number };
  components: Component[];
  wiring: Wire[];
}

interface Props {
  electronics: ElectronicsSection;
  className?: string;
}

const CircuitSchematic: React.FC<Props> = ({ electronics, className }) => {
  const { components, wiring } = electronics;

  // Simple layout logic: place components in a circle or grid
  const componentPositions = useMemo(() => components.reduce((acc, comp, i) => {
    const angle = (i / components.length) * 2 * Math.PI;
    acc[comp.component_id] = {
      x: 250 + 150 * Math.cos(angle),
      y: 200 + 150 * Math.sin(angle),
    };
    return acc;
  }, {} as Record<string, { x: number; y: number }>), [components]);

  return (
    <div className={cn("bg-slate-950 rounded-xl p-4 border border-slate-800 shadow-2xl relative overflow-hidden group", className)}>
      <div className="absolute inset-0 opacity-10 pointer-events-none" 
           style={{ backgroundImage: 'radial-gradient(#3b82f6 1px, transparent 1px)', backgroundSize: '20px 20px' }} 
      />
      
      <div className="flex items-center justify-between mb-4 relative z-10">
          <div className="flex items-center gap-2">
             <div className="size-2 rounded-full bg-blue-500 animate-pulse" />
             <h3 className="text-slate-200 text-[10px] font-black uppercase tracking-widest">Logic Schematic</h3>
          </div>
          <Badge variant="outline" className="text-[8px] border-blue-500/30 text-blue-400 bg-blue-500/5">
              TSC_ENGINE_V1
          </Badge>
      </div>

      <svg viewBox="0 0 500 400" className="w-full h-auto relative z-10 drop-shadow-2xl">
        <defs>
            <filter id="glow">
                <feGaussianBlur stdDeviation="2" result="coloredBlur"/>
                <feMerge>
                    <feMergeNode in="coloredBlur"/>
                    <feMergeNode in="SourceGraphic"/>
                </feMerge>
            </filter>
        </defs>

        {/* Draw Wires */}
        {wiring.map((wire) => {
          const start = componentPositions[wire.from.component];
          const end = componentPositions[wire.to.component];
          if (!start || !end) return null;

          return (
            <g key={wire.wire_id}>
                <line
                    x1={start.x}
                    y1={start.y}
                    x2={end.x}
                    y2={end.y}
                    stroke="#3b82f6"
                    strokeWidth="3"
                    strokeOpacity="0.1"
                />
                <line
                    x1={start.x}
                    y1={start.y}
                    x2={end.x}
                    y2={end.y}
                    stroke="#3b82f6"
                    strokeWidth="1.5"
                    className="opacity-80"
                    filter="url(#glow)"
                />
            </g>
          );
        })}

        {/* Draw Components */}
        {components.map((comp) => {
          const pos = componentPositions[comp.component_id];
          const isPSU = comp.type === 'power_supply';
          
          return (
            <g key={comp.component_id} transform={`translate(${pos.x}, ${pos.y})`} className="cursor-pointer hover:scale-110 transition-transform">
              <rect
                x="-35"
                y="-25"
                width="70"
                height="50"
                rx="8"
                fill={isPSU ? 'rgba(239, 68, 68, 0.1)' : 'rgba(30, 41, 59, 0.8)'}
                stroke={isPSU ? '#ef4444' : '#3b82f6'}
                strokeWidth="2"
                className="backdrop-blur-sm"
              />
              <text
                dy="-0.2em"
                textAnchor="middle"
                fill="#f8fafc"
                fontSize="9"
                fontWeight="900"
                className="select-none pointer-events-none uppercase tracking-tighter"
              >
                {comp.component_id}
              </text>
              <text
                y="18"
                textAnchor="middle"
                fill="#94a3b8"
                fontSize="7"
                fontWeight="700"
                className="select-none pointer-events-none uppercase tracking-widest opacity-60"
              >
                {comp.type}
              </text>
            </g>
          );
        })}
      </svg>
    </div>
  );
};

export default CircuitSchematic;
