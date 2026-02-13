import React from 'react';
import ModelViewer from './ModelViewer';

interface WireRoute {
  wire_id: string;
  waypoints: [number, number, number][];
  gauge_awg?: number;
}

interface Props {
  assetUrl?: string | null;
  wireRoutes: WireRoute[];
  className?: string;
}

const WireView: React.FC<Props> = ({ assetUrl, wireRoutes, className }) => {
  return (
    <div className={`relative bg-slate-900 rounded-lg overflow-hidden ${className}`}>
      <h3 className="absolute top-4 left-4 z-10 text-slate-200 text-sm font-semibold pointer-events-none">
        Wire Routing (3D)
      </h3>
      <ModelViewer 
        assetUrl={assetUrl} 
        wireRoutes={wireRoutes} 
        className="w-full h-[400px]" 
      />
      <div className="absolute bottom-4 right-4 z-10 flex space-x-2 pointer-events-none">
        <div className="flex items-center space-x-1">
          <div className="w-3 h-3 bg-[#ef4444] rounded-full" />
          <span className="text-[10px] text-slate-400">Power Wires</span>
        </div>
      </div>
    </div>
  );
};

export default WireView;
