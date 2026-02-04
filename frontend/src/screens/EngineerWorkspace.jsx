import React, { useState } from 'react';
import { Link, useLocation } from 'react-router-dom';

const EngineerWorkspace = () => {
    const location = useLocation();

    return (
        <div className="flex flex-col h-full">
            {/* Header */}
            <header className="flex shrink-0 items-center justify-between border-b border-border-dark px-4 h-14 bg-background-dark">
                <div className="flex items-center gap-6">
                    <Link to="/" className="flex items-center gap-2">
                        <div className="size-7 bg-primary rounded flex items-center justify-center">
                            <span className="material-symbols-outlined text-white text-xl">token</span>
                        </div>
                        <h2 className="text-white text-lg font-bold tracking-tight">Agentic CAD</h2>
                    </Link>
                    <nav className="flex items-center bg-panel-dark p-1 rounded-lg border border-border-dark">
                        <Link to="/benchmark" className={`px-4 py-1.5 text-xs font-bold rounded transition-all ${location.pathname === '/benchmark' ? 'bg-primary/10 text-white border border-primary' : 'text-text-secondary hover:text-white'}`}>
                            BENCHMARK GENERATION
                        </Link>
                        <Link to="/workspace" className={`px-4 py-1.5 text-xs font-bold rounded transition-all ${location.pathname === '/workspace' ? 'bg-primary/10 text-white border border-primary' : 'text-text-secondary hover:text-white'}`}>
                            ENGINEER WORKSPACE
                        </Link>
                    </nav>
                </div>
                <div className="flex items-center gap-4">
                    <div className="flex items-center gap-2 px-3 py-1 bg-green-500/10 border border-green-500/30 rounded-full">
                        <span className="relative flex h-2 w-2">
                            <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-green-400 opacity-75"></span>
                            <span className="relative inline-flex rounded-full h-2 w-2 bg-green-500"></span>
                        </span>
                        <span className="text-[10px] font-mono font-bold text-green-400 tracking-wider">SANDBOX: ISOLATED</span>
                    </div>
                    <div className="h-6 w-px bg-border-dark"></div>
                    <button className="flex items-center justify-center gap-2 rounded h-8 px-4 bg-primary hover:bg-blue-600 transition-colors text-white text-xs font-bold">
                        <span className="material-symbols-outlined text-base">rocket_launch</span>
                        SOLVE
                    </button>
                </div>
            </header>

            <div className="flex flex-1 overflow-hidden">
                {/* Sidebar */}
                <aside className="w-72 flex flex-col border-r border-border-dark bg-[#0c131a]">
                    <div className="flex-1 flex flex-col min-h-0 overflow-hidden">
                        <div className="p-4 border-b border-border-dark">
                            <h3 className="text-[10px] font-black uppercase tracking-widest text-text-secondary mb-3">Episode History</h3>
                             <div className="relative mb-3">
                                <span className="absolute left-2 top-1/2 -translate-y-1/2 material-symbols-outlined text-text-secondary text-sm">search</span>
                                <input className="w-full bg-panel-dark border border-border-dark rounded py-1 pl-7 text-xs text-white focus:outline-none focus:border-primary" placeholder="Filter..." type="text"/>
                            </div>
                        </div>
                        <div className="flex-1 overflow-y-auto">
                            <div className="p-3 bg-primary/5 border-l-2 border-primary cursor-pointer">
                                <div className="flex justify-between items-start mb-1">
                                    <span className="text-xs font-bold text-white">EP-442: Thermal Bracket</span>
                                    <span className="text-[9px] bg-primary text-white px-1 rounded">SOLVING</span>
                                </div>
                                <span className="text-[10px] text-text-secondary">4m ago • #Aerospace</span>
                            </div>
                             <div className="p-3 border-b border-border-dark/30 hover:bg-white/5 cursor-pointer">
                                <div className="flex justify-between items-start mb-1">
                                    <span className="text-xs font-medium text-text-secondary">EP-441: Motor Mount</span>
                                    <span className="material-symbols-outlined text-green-500 text-sm">check_circle</span>
                                </div>
                                <span className="text-[10px] text-text-secondary">1h ago • #Robotics</span>
                            </div>
                        </div>
                    </div>
                    <div className="h-64 flex flex-col border-t border-border-dark bg-panel-dark">
                        <div className="p-3 border-b border-border-dark flex items-center justify-between">
                            <h3 className="text-[10px] font-black uppercase tracking-widest text-primary">Skill Library</h3>
                            <span className="material-symbols-outlined text-sm text-text-secondary">school</span>
                        </div>
                        <div className="flex-1 overflow-y-auto p-2 space-y-1">
                            {['lattice_generator_v2', 'compliant_hinge_pt', 'multi_shell_union'].map(skill => (
                                <div key={skill} className="flex items-center gap-2 p-2 rounded hover:bg-white/5 cursor-pointer border border-transparent hover:border-border-dark group">
                                    <span className="material-symbols-outlined text-yellow-500 text-sm">dynamic_form</span>
                                    <span className="text-xs text-gray-300 group-hover:text-white">{skill}</span>
                                </div>
                            ))}
                        </div>
                    </div>
                </aside>

                {/* Main Content */}
                <main className="flex-1 flex flex-col min-w-0 bg-code-bg">
                    {/* Architect TODO */}
                    <section className="h-[35%] flex flex-col border-b border-border-dark bg-[#0b1016]">
                        <div className="px-4 py-2 border-b border-border-dark flex items-center gap-2">
                            <span className="material-symbols-outlined text-primary text-lg">architecture</span>
                            <span className="text-[10px] font-black uppercase tracking-widest text-text-secondary">Architect's TODO List</span>
                        </div>
                        <div className="flex-1 overflow-y-auto p-4 space-y-3">
                            <div className="flex items-center gap-3 bg-panel-dark/50 p-2 rounded border border-border-dark/50">
                                <span className="material-symbols-outlined text-green-500 text-xl">check_box</span>
                                <div className="flex-1">
                                    <div className="text-xs font-bold text-white">Define bounding volume and mounting holes</div>
                                    <div className="text-[10px] text-green-500 font-mono">Status: Verified</div>
                                </div>
                            </div>
                             <div className="flex items-center gap-3 bg-primary/5 p-2 rounded border border-primary/20">
                                <span className="material-symbols-outlined text-primary text-xl">indeterminate_check_box</span>
                                <div className="flex-1">
                                    <div className="text-xs font-bold text-white">Implement stress-relieving fillets on inner corners</div>
                                    <div className="text-[10px] text-primary font-mono">Status: In Progress</div>
                                </div>
                            </div>
                        </div>
                    </section>
                    {/* Code Section */}
                    <section className="flex-1 flex flex-col min-h-0">
                         <div className="flex items-center h-9 bg-[#0d1116] border-b border-border-dark">
                            <div className="flex items-center px-4 h-full bg-background-dark border-t-2 border-primary text-[11px] font-bold text-white">
                                <span className="material-symbols-outlined text-sm mr-2 text-blue-400">terminal</span>
                                impl_build123d.py
                            </div>
                        </div>
                        <div className="flex-1 flex font-mono text-sm overflow-hidden">
                             <div className="w-12 flex-shrink-0 bg-[#0d1116] text-gray-600 text-right pr-3 pt-4 select-none border-r border-border-dark/30 leading-6 text-xs">
                                {Array.from({length: 12}, (_, i) => <div key={i}>{i+1}</div>)}
                            </div>
                             <div className="flex-1 pl-4 pt-4 text-gray-300 leading-6 text-xs overflow-auto">
                                <div><span className="syntax-keyword">from</span> build123d <span className="syntax-keyword">import</span> *</div>
                                <div><span className="syntax-keyword">with</span> <span className="syntax-class">BuildPart</span>() <span className="syntax-keyword">as</span> bracket:</div>
                                <div className="pl-4"> <span className="syntax-keyword">with</span> <span className="syntax-class">BuildSketch</span>() <span className="syntax-keyword">as</span> sk:</div>
                                <div className="pl-8"> <span className="syntax-class">Rectangle</span>(width=60, height=80)</div>
                                <div className="pl-4"> <span className="syntax-func">extrude</span>(amount=10)</div>
                                <br/>
                                <div className="pl-4 relative bg-primary/10 -mx-4 px-4 border-l-2 border-primary">
                                    <span className="absolute right-4 top-0 text-[9px] text-primary/50 font-bold">RECOVERED FROM SKILL LIB</span>
                                    <span className="syntax-func">add_mounting_holes</span>(diameter=6.5, pattern=<span className="syntax-string">"grid"</span>)
                                </div>
                                <div className="pl-4"> <span className="syntax-keyword">return</span> bracket</div>
                            </div>
                        </div>
                    </section>
                </main>

                {/* Right Pane (Stats & Logs) */}
                <aside className="w-[480px] flex flex-col border-l border-border-dark bg-panel-dark">
                    <div className="h-[55%] relative bg-gradient-to-br from-[#1a222c] to-[#0a0f14] flex flex-col overflow-hidden items-center justify-center">
                         <div className="absolute top-4 right-4 z-10 w-44 bg-black/60 backdrop-blur-md rounded border border-white/10 p-3 space-y-2">
                             <div className="flex justify-between items-center"><span className="text-[10px] text-text-secondary">Cost (Est.)</span><span className="text-xs font-mono text-white">$4.82</span></div>
                             <div className="flex justify-between items-center"><span className="text-[10px] text-text-secondary">Dfm Score</span><span className="text-xs font-mono text-green-400">92%</span></div>
                         </div>
                         <div className="text-white opacity-20 text-4xl font-black">PREVIEW</div>
                    </div>
                    <div className="flex-1 flex flex-col border-t border-border-dark overflow-hidden">
                         <div className="flex border-b border-border-dark h-8">
                             <div className="px-4 flex items-center gap-2 border-r border-border-dark bg-background-dark text-[10px] font-bold text-white">
                                <span className="material-symbols-outlined text-sm text-blue-400">terminal</span> CONTAINER LOGS
                            </div>
                         </div>
                         <div className="flex-1 flex flex-col bg-[#0a0f14] overflow-hidden">
                             <div className="flex-1 p-3 font-mono text-[11px] overflow-y-auto space-y-1">
                                <div className="text-gray-500">[0.00s] Initializing build123d kernel...</div>
                                <div className="text-green-500">[OK] BuildPart created. Vol: 12.4cm³</div>
                                <div className="text-yellow-400">[WARN] Hole clearance near edge &lt; 2.0mm</div>
                            </div>
                         </div>
                    </div>
                </aside>
            </div>
        </div>
    );
};

export default EngineerWorkspace;
