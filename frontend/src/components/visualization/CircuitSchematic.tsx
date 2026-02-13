import React from 'react';

interface Component {
  component_id: str;
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
  const componentPositions = components.reduce((acc, comp, i) => {
    const angle = (i / components.length) * 2 * Math.PI;
    acc[comp.component_id] = {
      x: 250 + 150 * Math.cos(angle),
      y: 200 + 150 * Math.sin(angle),
    };
    return acc;
  }, {} as Record<string, { x: number; y: number }>);

  return (
    <div className={`bg-slate-900 rounded-lg p-4 ${className}`}>
      <h3 className="text-slate-200 text-sm font-semibold mb-4">Circuit Schematic</h3>
      <svg viewBox="0 0 500 400" className="w-full h-auto">
        {/* Draw Wires */}
        {wiring.map((wire) => {
          const start = componentPositions[wire.from.component];
          const end = componentPositions[wire.to.component];
          if (!start || !end) return null;

          return (
            <line
              key={wire.wire_id}
              x1={start.x}
              y1={start.y}
              x2={end.x}
              y2={end.y}
              stroke="#3b82f6"
              strokeWidth="2"
              className="opacity-60"
            />
          );
        })}

        {/* Draw Components */}
        {components.map((comp) => {
          const pos = componentPositions[comp.component_id];
          const isPSU = comp.type === 'power_supply';
          
          return (
            <g key={comp.component_id} transform={`translate(${pos.x}, ${pos.y})`}>
              <rect
                x="-30"
                y="-20"
                width="60"
                height="40"
                rx="4"
                fill={isPSU ? '#ef4444' : '#1e293b'}
                stroke="#64748b"
                strokeWidth="2"
              />
              <text
                dy="0.3em"
                textAnchor="middle"
                fill="#f8fafc"
                fontSize="10"
                className="select-none pointer-events-none"
              >
                {comp.component_id}
              </text>
              <text
                y="25"
                textAnchor="middle"
                fill="#94a3b8"
                fontSize="8"
                className="select-none pointer-events-none"
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
