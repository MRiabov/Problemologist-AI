import React, { useState, useEffect } from 'react';
import { Link } from 'react-router-dom';
import client from '../api/client';

const IdeDashboard = () => {
    // State placeholders for real data
    const [episodes, setEpisodes] = useState([]);
    const [selectedEpisode, setSelectedEpisode] = useState(null);
    const [codeContent, setCodeContent] = useState(`import cad_library as cad
from agents.physics import ConstraintSolver

# Initialize the main assembly context
def optimize_bracket_structure(params):
    solver = ConstraintSolver(tolerance=1e-5)
    base_plate = cad.create_box(width=params['w'], height=10, depth=50)

    # Agent generated adjustment based on interference
    if solver.check_collision(base_plate):
         params['w'] += 2.5 # Expanding width to resolve conflict
    return base_plate`);

    useEffect(() => {
        // Fetch episodes
        client.get('/episodes').then(res => {
            setEpisodes(res.data);
        }).catch(err => console.error("Failed to fetch episodes", err));
    }, []);

    return (
        <div className="flex flex-col h-full">
            {/* Header */}
            <header className="flex shrink-0 items-center justify-between whitespace-nowrap border-b border-solid border-border-dark px-4 h-12 bg-[#101a23]">
                <div className="flex items-center gap-4 text-white">
                    <div className="size-6 text-primary flex items-center justify-center">
                        <span className="material-symbols-outlined text-2xl">grid_view</span>
                    </div>
                    <h2 className="text-white text-lg font-bold leading-tight tracking-tight">CAD-Agent IDE <span className="text-text-secondary font-normal text-sm ml-2">v3.0.1-beta</span></h2>
                </div>
                <div className="flex flex-1 justify-center max-w-xl mx-auto">
                    <div className="flex items-center bg-panel-dark rounded-md px-2 py-1 border border-border-dark w-full max-w-md">
                        <span className="material-symbols-outlined text-text-secondary text-lg mr-2">search</span>
                        <input className="bg-transparent border-none text-sm w-full text-white placeholder-text-secondary focus:ring-0 outline-none" placeholder="Search files, symbols, or agents..." type="text" />
                        <div className="flex gap-1">
                            <kbd className="hidden sm:inline-block px-1.5 py-0.5 text-xs text-text-secondary bg-border-dark rounded border border-[#304d63]">⌘K</kbd>
                        </div>
                    </div>
                </div>
                <div className="flex items-center gap-3">
                    <div className="flex items-center gap-4 mr-4">
                        <Link to="/workspace" className="text-text-secondary hover:text-white text-sm font-medium transition-colors">Workspace</Link>
                        <Link to="/benchmark" className="text-text-secondary hover:text-white text-sm font-medium transition-colors">Benchmark</Link>
                        <a className="text-text-secondary hover:text-white text-sm font-medium transition-colors" href="#">Help</a>
                    </div>
                    <button className="flex items-center justify-center gap-2 rounded-md h-8 px-4 bg-primary hover:bg-blue-600 transition-colors text-white text-xs font-bold uppercase tracking-wider">
                        <span className="material-symbols-outlined text-base">play_arrow</span>
                        Run
                    </button>
                    <div className="w-8 h-8 rounded-full bg-gradient-to-tr from-purple-500 to-blue-500 border border-white/10"></div>
                </div>
            </header>

            {/* Main Workspace */}
            <div className="flex flex-1 overflow-hidden">
                {/* Left Sidebar: Episode History */}
                <aside className="w-72 flex flex-col border-r border-border-dark bg-[#121d28]">
                    <div className="p-4 border-b border-border-dark space-y-3">
                        <div className="flex items-center justify-between">
                            <h3 className="text-sm font-bold uppercase tracking-wider text-text-secondary">Episode History</h3>
                            <span className="material-symbols-outlined text-text-secondary text-sm cursor-pointer hover:text-white">filter_list</span>
                        </div>
                        <div className="relative">
                            <span className="absolute left-2 top-1/2 -translate-y-1/2 material-symbols-outlined text-text-secondary text-lg">search</span>
                            <input className="w-full bg-panel-dark border border-border-dark rounded-md py-1.5 pl-8 pr-3 text-sm text-white focus:outline-none focus:border-primary" placeholder="Filter episodes..." type="text" />
                        </div>
                        <div className="flex flex-wrap gap-2">
                            <span className="px-2 py-0.5 rounded text-xs font-medium bg-primary/20 text-primary border border-primary/30 cursor-pointer">All</span>
                            <span className="px-2 py-0.5 rounded text-xs font-medium bg-panel-dark text-text-secondary border border-border-dark hover:border-text-secondary cursor-pointer">#Kinematic</span>
                            <span className="px-2 py-0.5 rounded text-xs font-medium bg-panel-dark text-text-secondary border border-border-dark hover:border-text-secondary cursor-pointer">#Spatial</span>
                        </div>
                    </div>
                    <div className="flex-1 overflow-y-auto">
                         {/* Mock Active Item */}
                        <div className="group flex flex-col p-3 border-l-2 border-primary bg-primary/5 hover:bg-primary/10 cursor-pointer transition-colors">
                            <div className="flex justify-between items-start mb-1">
                                <span className="text-sm font-semibold text-white">Ep-1025: Gear Assembly</span>
                                <div className="flex items-center gap-1 text-[10px] text-primary bg-primary/20 px-1.5 py-0.5 rounded">
                                    <span className="block w-1.5 h-1.5 rounded-full bg-primary animate-pulse"></span>
                                    RUNNING
                                </div>
                            </div>
                            <div className="flex items-center justify-between text-xs text-text-secondary">
                                <span>Just now</span>
                                <div className="flex gap-1">
                                    <span className="text-[10px] bg-border-dark px-1 rounded">#Kinematic</span>
                                </div>
                            </div>
                        </div>
                         {/* Mock History Items */}
                         {[1024, 1023, 1022].map((id, idx) => (
                             <div key={id} className="group flex flex-col p-3 border-l-2 border-transparent hover:bg-panel-dark cursor-pointer transition-colors border-b border-border-dark/50">
                                <div className="flex justify-between items-start mb-1">
                                    <span className="text-sm font-medium text-text-secondary group-hover:text-white">Ep-{id}: Test Case</span>
                                    <span className={`material-symbols-outlined text-sm ${idx === 1 ? 'text-red-500' : 'text-green-500'}`}>
                                        {idx === 1 ? 'cancel' : 'check_circle'}
                                    </span>
                                </div>
                                <div className="flex items-center justify-between text-xs text-text-secondary">
                                    <span>{idx + 1}h ago</span>
                                    <div className="flex gap-1">
                                        <span className="text-[10px] bg-border-dark px-1 rounded">#Tag</span>
                                    </div>
                                </div>
                            </div>
                         ))}
                    </div>
                </aside>

                {/* Middle Pane */}
                <main className="flex-1 flex flex-col min-w-0 border-r border-border-dark bg-[#101a23]">
                    {/* Editor Tabs */}
                    <div className="flex items-center h-9 bg-[#0d1116] border-b border-border-dark overflow-x-auto no-scrollbar">
                        <div className="flex items-center px-4 h-full bg-[#101a23] border-t-2 border-primary text-xs font-medium text-white min-w-[120px]">
                            <span className="material-symbols-outlined text-sm mr-2 text-blue-400">code</span>
                            generator.py
                            <span className="material-symbols-outlined text-xs ml-auto hover:text-white cursor-pointer text-text-secondary">close</span>
                        </div>
                        <div className="flex items-center px-4 h-full text-text-secondary hover:bg-[#16202a] cursor-pointer text-xs font-medium min-w-[120px] border-r border-border-dark/50">
                            <span className="material-symbols-outlined text-sm mr-2 text-yellow-500">description</span>
                            specs.json
                        </div>
                         <div className="flex items-center px-4 h-full text-text-secondary hover:bg-[#16202a] cursor-pointer text-xs font-medium min-w-[120px] border-r border-border-dark/50">
                            <span className="material-symbols-outlined text-sm mr-2 text-purple-400">smart_toy</span>
                            agent_config.yaml
                        </div>
                    </div>
                    {/* Code Editor */}
                    <div className="flex-1 relative bg-code-bg font-mono text-sm overflow-auto">
                        <div className="flex min-h-full">
                            <div className="w-12 flex-shrink-0 bg-[#0d1116] text-gray-600 text-right pr-3 pt-4 select-none border-r border-border-dark/30 leading-6">
                                {Array.from({length: 15}, (_, i) => <div key={i}>{i+1}</div>)}
                            </div>
                            <div className="flex-1 pl-4 pt-4 text-gray-300 leading-6 whitespace-pre-wrap">
                                {/* Simulating Syntax Highlighting with basic logic */}
                                {codeContent.split('\n').map((line, i) => (
                                    <div key={i}>
                                        {line.includes('def ') ? (
                                            <>
                                                <span className="syntax-keyword">def</span> <span className="syntax-func">{line.replace('def ', '').split('(')[0]}</span>{line.split('(')[1] ? '(' + line.split('(')[1] : ''}
                                            </>
                                        ) : line}
                                    </div>
                                ))}
                            </div>
                        </div>
                    </div>
                    {/* Agent Cognition Stream */}
                    <div className="h-[35%] border-t border-border-dark flex flex-col bg-[#11161d]">
                        <div className="px-3 py-2 border-b border-border-dark flex justify-between items-center bg-[#16202a]">
                            <div className="flex items-center gap-2">
                                <span className="material-symbols-outlined text-purple-400 text-lg">psychology</span>
                                <span className="text-xs font-bold uppercase tracking-wider text-text-secondary">Agent Cognition Stream</span>
                            </div>
                            <span className="px-2 py-0.5 rounded-full bg-green-500/20 text-green-400 text-[10px] border border-green-500/30">Thinking</span>
                        </div>
                        <div className="flex-1 overflow-y-auto p-4 space-y-4">
                            <div className="flex gap-3 opacity-60">
                                <div className="w-6 flex-shrink-0 flex flex-col items-center">
                                    <div className="w-6 h-6 rounded-full bg-border-dark flex items-center justify-center">
                                        <span className="material-symbols-outlined text-xs text-text-secondary">visibility</span>
                                    </div>
                                    <div className="w-0.5 h-full bg-border-dark/50 my-1"></div>
                                </div>
                                <div className="flex-1">
                                    <p className="text-xs text-text-secondary mb-1">Visual Analysis</p>
                                    <div className="bg-panel-dark p-3 rounded-lg border border-border-dark text-sm text-gray-400">
                                        Detected 2 potential collision points on the Z-axis extrusion.
                                    </div>
                                </div>
                            </div>
                             <div className="flex gap-3">
                                <div className="w-6 flex-shrink-0 flex flex-col items-center">
                                    <div className="w-6 h-6 rounded-full bg-purple-500/20 flex items-center justify-center border border-purple-500/50">
                                        <span className="material-symbols-outlined text-xs text-purple-400">lightbulb</span>
                                    </div>
                                </div>
                                <div className="flex-1">
                                    <p className="text-xs text-purple-400 mb-1 font-medium">Reasoning Step</p>
                                    <div className="bg-[#1e1e2e] p-3 rounded-lg border border-purple-500/30 text-sm text-gray-200 shadow-lg shadow-purple-900/10">
                                        <p>The collision implies the bracket is too narrow for the bearing housing. I will attempt to increase the <code className="bg-black/30 px-1 rounded text-purple-300">width</code> parameter by 2.5mm and re-run the solver check.</p>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </main>

                {/* Right Pane */}
                <aside className="w-[420px] flex flex-col border-l border-border-dark bg-[#0f151b]">
                    {/* 3D Viewport */}
                    <div className="h-[60%] relative bg-gradient-to-br from-[#1a2634] to-[#0f151b] flex items-center justify-center overflow-hidden">
                        <div className="absolute top-4 left-4 z-10 flex flex-col gap-2 bg-black/40 backdrop-blur-sm p-1 rounded-md border border-white/5">
                            {['3d_rotation', 'pan_tool', 'zoom_in', 'grid_4x4'].map(icon => (
                                <button key={icon} className="w-8 h-8 flex items-center justify-center rounded hover:bg-white/10 text-white/80">
                                    <span className="material-symbols-outlined text-lg">{icon}</span>
                                </button>
                            ))}
                        </div>
                         {/* Metrics */}
                        <div className="absolute top-4 right-4 z-10 w-48 bg-black/60 backdrop-blur-md rounded-lg border border-border-dark shadow-xl overflow-hidden">
                            <div className="px-3 py-2 border-b border-white/5 bg-white/5">
                                <h4 className="text-xs font-bold text-white uppercase">Live Metrics</h4>
                            </div>
                            <div className="p-3 space-y-3">
                                <div className="flex justify-between items-center">
                                    <span className="text-xs text-text-secondary flex items-center gap-1"><span className="material-symbols-outlined text-yellow-400 text-[14px]">bolt</span> Energy</span>
                                    <span className="text-sm font-mono text-white">420kJ</span>
                                </div>
                                <div className="flex justify-between items-center">
                                    <span className="text-xs text-text-secondary flex items-center gap-1"><span className="material-symbols-outlined text-green-400 text-[14px]">attach_money</span> Cost</span>
                                    <span className="text-sm font-mono text-white">$0.04</span>
                                </div>
                            </div>
                        </div>
                        {/* 3D Content Placeholder */}
                        <div className="text-white opacity-20 text-4xl font-black">3D VIEW</div>
                    </div>
                    {/* Execution Log */}
                    <div className="flex-1 border-t border-border-dark bg-[#0a0e12] flex flex-col min-h-0">
                         <div className="px-3 py-1.5 border-b border-border-dark flex justify-between items-center bg-[#16202a]">
                            <div className="flex items-center gap-2">
                                <span className="material-symbols-outlined text-text-secondary text-sm">terminal</span>
                                <span className="text-xs font-bold uppercase tracking-wider text-text-secondary">Execution Log</span>
                            </div>
                        </div>
                        <div className="flex-1 p-3 font-mono text-xs overflow-y-auto space-y-1">
                            <div className="text-gray-500"><span className="text-blue-500">[INFO]</span> Environment initialized.</div>
                            <div className="text-gray-400"><span className="text-purple-400">[AGENT]</span> Starting geometry generation.</div>
                            <div className="text-green-500/90"><span className="font-bold">[SUCCESS]</span> Optimization complete.</div>
                            <div className="text-gray-500 animate-pulse">_</div>
                        </div>
                    </div>
                </aside>
            </div>
        </div>
    );
};

export default IdeDashboard;
