import React from 'react';

interface Event {
  timestamp: number;
  motor_states: Record<string, 'on' | 'off'>;
}

interface Props {
  events: Event[];
  className?: string;
}

const CircuitTimeline: React.FC<Props> = ({ events, className }) => {
  if (!events || events.length === 0) return null;

  const motors = Object.keys(events[0].motor_states);
  const totalDuration = events[events.length - 1].timestamp - events[0].timestamp;

  return (
    <div className={`bg-slate-900 rounded-lg p-4 ${className}`}>
      <h3 className="text-slate-200 text-sm font-semibold mb-4">Motor Power Timeline</h3>
      <div className="space-y-4">
        {motors.map((motorId) => (
          <div key={motorId} className="flex items-center space-x-4">
            <span className="text-xs text-slate-400 w-24 truncate">{motorId}</span>
            <div className="flex-1 h-4 bg-slate-800 rounded relative overflow-hidden">
              {events.map((event, i) => {
                if (i === events.length - 1) return null;
                const nextEvent = events[i + 1];
                const start = ((event.timestamp - events[0].timestamp) / totalDuration) * 100;
                const width = ((nextEvent.timestamp - event.timestamp) / totalDuration) * 100;
                const isOn = event.motor_states[motorId] === 'on';

                return (
                  <div
                    key={i}
                    className={`absolute h-full ${isOn ? 'bg-green-500' : 'bg-transparent'}`}
                    style={{ left: `${start}%`, width: `${width}%` }}
                  />
                );
              })}
            </div>
          </div>
        ))}
      </div>
      <div className="mt-2 flex justify-between text-[10px] text-slate-500">
        <span>0s</span>
        <span>{totalDuration.toFixed(1)}s</span>
      </div>
    </div>
  );
};

export default CircuitTimeline;
