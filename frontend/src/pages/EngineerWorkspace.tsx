import { useNavigate } from 'react-router-dom';
import { useEffect, useState } from 'react';
import { fetchEpisodes, fetchSkills, runAgent } from '../api/client';
import type { Episode, Skill } from '../api/types';

export default function EngineerWorkspace() {
  const navigate = useNavigate();
  const [episodes, setEpisodes] = useState<Episode[]>([]);
  const [skills, setSkills] = useState<Skill[]>([]);
  const [loading, setLoading] = useState(true);
  const [taskPrompt, setTaskPrompt] = useState('');
  const [running, setRunning] = useState(false);
  const [selectedEpisode, setSelectedEpisode] = useState<Episode | null>(null);

  useEffect(() => {
    async function loadData() {
        try {
            const [episodesData, skillsData] = await Promise.all([
                fetchEpisodes(),
                fetchSkills()
            ]);
            setEpisodes(episodesData);
            setSkills(skillsData);
        } catch (e) {
            console.error("Failed to load data", e);
        } finally {
            setLoading(false);
        }
    }
    loadData();
  }, []);

  const handleSelectEpisode = async (ep: Episode) => {
    try {
        const fullEp = await fetchEpisode(ep.id);
        setSelectedEpisode(fullEp);
    } catch (e) {
        console.error("Failed to fetch episode details", e);
        setSelectedEpisode(ep);
    }
  };

  const handleRunAgent = async () => {
    if (!taskPrompt) return;
    setRunning(true);
    try {
        const sessionId = `sess-${Math.random().toString(36).substring(2, 10)}`;
        await runAgent(taskPrompt, sessionId);
        // Refresh episodes
        const episodesData = await fetchEpisodes();
        setEpisodes(episodesData);
        setTaskPrompt('');
    } catch (e) {
        console.error("Failed to run agent", e);
    } finally {
        setRunning(false);
    }
  };

  return (
    <div className="flex flex-col h-screen overflow-hidden">
        {/* Header */}
        <header className="flex shrink-0 items-center justify-between border-b border-border-dark px-4 h-14 bg-background-dark">
            <div className="flex items-center gap-6">
                <div className="flex items-center gap-2">
                    <div className="size-7 bg-primary rounded flex items-center justify-center">
                        <span className="material-symbols-outlined text-white text-xl">token</span>
                    </div>
                    <h2 className="text-white text-lg font-bold tracking-tight">Agentic CAD</h2>
                </div>
                <nav className="flex items-center bg-panel-dark p-1 rounded-lg border border-border-dark">
                    <button
                        onClick={() => navigate('/benchmark')}
                        className="px-4 py-1.5 text-xs font-bold rounded tab-inactive transition-all hover:bg-white/5 text-text-secondary"
                    >
                        BENCHMARK GENERATION
                    </button>
                    <button className="px-4 py-1.5 text-xs font-bold rounded tab-active transition-all border-primary bg-primary/10 text-white shadow-sm">
                        ENGINEER WORKSPACE
                    </button>
                </nav>
            </div>
            {/* Rest of Header */}
            <div className="flex items-center gap-4 flex-1 justify-end">
                <div className="flex items-center bg-panel-dark rounded-md px-2 py-1 border border-border-dark w-full max-w-sm">
                    <input 
                        className="bg-transparent border-none text-xs w-full text-white placeholder-text-secondary focus:ring-0 focus:outline-none" 
                        placeholder="Describe the task for the engineer..." 
                        type="text"
                        value={taskPrompt}
                        onChange={(e) => setTaskPrompt(e.target.value)}
                        onKeyDown={(e) => e.key === 'Enter' && handleRunAgent()}
                    />
                </div>
                <div className="flex items-center gap-2 px-3 py-1 bg-green-500/10 border border-green-500/30 rounded-full shrink-0">
                    <span className="relative flex h-2 w-2">
                        <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-green-400 opacity-75"></span>
                        <span className="relative inline-flex rounded-full h-2 w-2 bg-green-500"></span>
                    </span>
                    <span className="text-[10px] font-mono font-bold text-green-400 tracking-wider">SANDBOX: ISOLATED</span>
                </div>
                <div className="h-6 w-px bg-border-dark shrink-0"></div>
                <button
                    onClick={handleRunAgent}
                    disabled={running || !taskPrompt}
                    className="flex items-center justify-center gap-2 rounded h-8 px-4 bg-primary hover:bg-blue-600 transition-colors text-white text-xs font-bold disabled:opacity-50 disabled:cursor-not-allowed shrink-0"
                >
                    <span className="material-symbols-outlined text-base">rocket_launch</span>
                    {running ? 'RUNNING...' : 'SOLVE'}
                </button>
            </div>
        </header>

        {/* Main Content */}
        <div className="flex flex-1 overflow-hidden">
            {/* Left Sidebar */}
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
                        {loading ? (
                             <div className="p-4 text-xs text-text-secondary">Loading episodes...</div>
                        ) : episodes.length === 0 ? (
                             <div className="p-4 text-xs text-text-secondary">No episodes found.</div>
                        ) : (
                            episodes.map(ep => (
                                <div 
                                    key={ep.id} 
                                    onClick={() => handleSelectEpisode(ep)}
                                    className={`p-3 border-b border-border-dark/30 hover:bg-white/5 cursor-pointer transition-colors ${selectedEpisode?.id === ep.id ? 'bg-primary/10 border-l-2 border-l-primary' : ''}`}
                                >
                                    <div className="flex justify-between items-start mb-1">
                                        <span className={`text-xs font-medium ${selectedEpisode?.id === ep.id ? 'text-white' : 'text-gray-400'}`}>{ep.task.substring(0, 24)}...</span>
                                        {ep.status === 'running' ? (
                                            <span className="text-[9px] bg-primary text-white px-1 rounded">RUNNING</span>
                                        ) : ep.status === 'completed' || ep.status === 'success' ? ( // Handle mapped status
                                            <span className="material-symbols-outlined text-green-500 text-sm">check_circle</span>
                                        ) : (
                                            <span className="material-symbols-outlined text-red-500 text-sm">cancel</span>
                                        )}
                                    </div>
                                    <span className="text-[10px] text-text-secondary">{new Date(ep.created_at).toLocaleTimeString()} • {ep.id.substring(0,8)}</span>
                                </div>
                            ))
                        )}
                    </div>
                </div>
                <div className="h-64 flex flex-col border-t border-border-dark bg-panel-dark">
                    <div className="p-3 border-b border-border-dark flex items-center justify-between">
                        <h3 className="text-[10px] font-black uppercase tracking-widest text-primary">Skill Library</h3>
                        <span className="material-symbols-outlined text-sm text-text-secondary">school</span>
                    </div>
                    <div className="flex-1 overflow-y-auto p-2 space-y-1">
                        {loading ? (
                            <div className="p-2 text-xs text-text-secondary">Loading skills...</div>
                        ) : (
                            skills.map(skill => (
                                <div key={skill.name} className="flex items-center gap-2 p-2 rounded hover:bg-white/5 cursor-pointer border border-transparent hover:border-border-dark group">
                                    <span className="material-symbols-outlined text-blue-400 text-sm">code</span>
                                    <span className="text-xs text-gray-300 group-hover:text-white">{skill.name}</span>
                                </div>
                            ))
                        )}
                    </div>
                </div>
            </aside>

            {/* Middle Content */}
            <main className="flex-1 flex flex-col min-w-0 bg-code-bg">
                <section className="h-[35%] flex flex-col border-b border-border-dark bg-[#0b1016]">
                    <div className="px-4 py-2 border-b border-border-dark flex items-center justify-between">
                        <div className="flex items-center gap-2">
                            <span className="material-symbols-outlined text-primary text-lg">architecture</span>
                            <span className="text-[10px] font-black uppercase tracking-widest text-text-secondary">Architect's TODO List</span>
                        </div>
                        {selectedEpisode && (
                            <span className="text-[10px] font-mono text-primary truncate max-w-md">
                                {selectedEpisode.task}
                            </span>
                        )}
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
                        <div className="flex items-center gap-3 bg-white/5 p-2 rounded border border-white/10 opacity-60">
                            <span className="material-symbols-outlined text-text-secondary text-xl">check_box_outline_blank</span>
                            <div className="flex-1">
                                <div className="text-xs font-bold text-white">Optimise mass for 3D printing (hollowing)</div>
                                <div className="text-[10px] text-text-secondary font-mono">Status: Pending</div>
                            </div>
                        </div>
                        <div className="flex items-center gap-3 bg-red-500/5 p-2 rounded border border-red-500/20">
                            <span className="material-symbols-outlined text-red-500 text-xl">cancel</span>
                            <div className="flex-1">
                                <div className="text-xs font-bold text-white">Apply 20mm uniform thickness</div>
                                <div className="text-[10px] text-red-500 font-mono">Status: Refused (Violates weight constraint)</div>
                            </div>
                        </div>
                    </div>
                </section>
                <section className="flex-1 flex flex-col min-h-0">
                    <div className="flex items-center h-9 bg-[#0d1116] border-b border-border-dark">
                        <div className="flex items-center px-4 h-full bg-background-dark border-t-2 border-primary text-[11px] font-bold text-white">
                            <span className="material-symbols-outlined text-sm mr-2 text-blue-400">terminal</span>
                            impl_build123d.py
                        </div>
                        <div className="flex items-center px-4 h-full text-text-secondary hover:bg-white/5 cursor-pointer text-[11px] border-r border-border-dark/50">
                            parameters.yaml
                        </div>
                    </div>
                    <div className="flex-1 flex font-mono text-sm overflow-hidden">
                        <div className="w-12 flex-shrink-0 bg-[#0d1116] text-gray-600 text-right pr-3 pt-4 select-none border-r border-border-dark/30 leading-6 text-xs">
                            1<br/>2<br/>3<br/>4<br/>5<br/>6<br/>7<br/>8<br/>9<br/>10<br/>11<br/>12
                        </div>
                        <div className="flex-1 pl-4 pt-4 text-gray-300 leading-6 text-xs overflow-auto">
                            <div><span className="syntax-keyword">from</span> build123d <span className="syntax-keyword">import</span> *</div>
                            <div><span className="syntax-keyword">with</span> <span className="syntax-class">BuildPart</span>() <span className="syntax-keyword">as</span> bracket:</div>
                            <div className="pl-4"> <span className="syntax-keyword">with</span> <span className="syntax-class">BuildSketch</span>() <span className="syntax-keyword">as</span> sk:</div>
                            <div className="pl-8"> <span className="syntax-class">Rectangle</span>(width=60, height=80)</div>
                            <div className="pl-8"> <span className="syntax-func">fillet</span>(sk.vertices(), radius=5)</div>
                            <div className="pl-4"> <span className="syntax-func">extrude</span>(amount=10)</div>
                            <br/>
                            <div className="pl-4"><span className="syntax-comment"># Learned pattern: apply_bolted_flange</span></div>
                            <div className="pl-4 relative bg-primary/10 -mx-4 px-4 border-l-2 border-primary">
                                <span className="absolute right-4 top-0 text-[9px] text-primary/50 font-bold">RECOVERED FROM SKILL LIB</span>
                                <span className="syntax-func">add_mounting_holes</span>(diameter=6.5, pattern=<span className="syntax-string">"grid"</span>)
                            </div>
                            <div className="pl-4"> <span className="syntax-comment"># Validating structural integrity...</span></div>
                            <div className="pl-4"> <span className="syntax-keyword">return</span> bracket</div>
                        </div>
                    </div>
                </section>
            </main>

            {/* Right Sidebar */}
            <aside className="w-[480px] flex flex-col border-l border-border-dark bg-panel-dark">
                <div className="h-[55%] relative bg-gradient-to-br from-[#1a222c] to-[#0a0f14] flex flex-col overflow-hidden">
                    <div className="absolute top-4 right-4 z-10 w-44 bg-black/60 backdrop-blur-md rounded border border-white/10 p-3 space-y-2">
                        <div className="flex justify-between items-center">
                            <span className="text-[10px] text-text-secondary">Cost (Est.)</span>
                            <span className="text-xs font-mono text-white">$4.82</span>
                        </div>
                        <div className="flex justify-between items-center">
                            <span className="text-[10px] text-text-secondary">Weight</span>
                            <span className="text-xs font-mono text-white">240g</span>
                        </div>
                        <div className="flex justify-between items-center">
                            <span className="text-[10px] text-text-secondary">Dfm Score</span>
                            <span className="text-xs font-mono text-green-400">92%</span>
                        </div>
                    </div>
                    <div className="absolute bottom-4 left-4 z-10 flex gap-2">
                        <button className="w-8 h-8 bg-black/40 rounded flex items-center justify-center border border-white/10 text-white/70 hover:text-white"><span className="material-symbols-outlined text-sm">view_in_ar</span></button>
                        <button className="w-8 h-8 bg-black/40 rounded flex items-center justify-center border border-white/10 text-white/70 hover:text-white"><span className="material-symbols-outlined text-sm">grid_on</span></button>
                    </div>
                    <div className="flex-1 flex items-center justify-center overflow-hidden">
                        {selectedEpisode?.assets && selectedEpisode.assets.filter(a => a.asset_type === 'video' || a.asset_type === 'image').length > 0 ? (
                            <div className="w-full h-full flex items-center justify-center p-4">
                                {selectedEpisode.assets.find(a => a.asset_type === 'video') ? (
                                    <video 
                                        src={selectedEpisode.assets.find(a => a.asset_type === 'video')?.s3_path} 
                                        controls 
                                        className="max-w-full max-h-full rounded shadow-2xl border border-white/10"
                                    />
                                ) : (
                                    <img 
                                        src={selectedEpisode.assets.find(a => a.asset_type === 'image')?.s3_path} 
                                        className="max-w-full max-h-full object-contain rounded shadow-2xl border border-white/10"
                                    />
                                )}
                            </div>
                        ) : (
                            <div className="relative w-48 h-48 border border-primary/20 rounded-xl flex items-center justify-center">
                                <div className="absolute inset-0 bg-primary/5 blur-xl"></div>
                                <div className="w-32 h-20 bg-primary/20 border-2 border-primary/40 rounded transform -rotate-12 shadow-2xl flex items-center justify-center">
                                    <span className="text-[10px] font-black tracking-tighter text-primary/60">NO_ASSET</span>
                                </div>
                                <div className="absolute w-full h-full border border-white/5 rounded-full scale-125"></div>
                            </div>
                        )}
                    </div>
                </div>
                <div className="flex-1 flex flex-col border-t border-border-dark overflow-hidden">
                    <div className="flex border-b border-border-dark h-8">
                        <div className="px-4 flex items-center gap-2 border-r border-border-dark bg-background-dark text-[10px] font-bold text-white">
                            <span className="material-symbols-outlined text-sm text-blue-400">terminal</span> CONTAINER LOGS
                        </div>
                        <div className="px-4 flex items-center gap-2 border-r border-border-dark text-[10px] font-bold text-text-secondary hover:text-white cursor-pointer">
                            <span className="material-symbols-outlined text-sm">psychology</span> CRITIC FEEDBACK
                        </div>
                    </div>
                    <div className="flex-1 flex flex-col bg-[#0a0f14] overflow-hidden">
                        <div className="flex-1 p-3 font-mono text-[11px] overflow-y-auto space-y-1">
                            {selectedEpisode?.traces && selectedEpisode.traces.length > 0 ? (
                                selectedEpisode.traces.map(trace => (
                                    <div key={trace.id} className="space-y-1 mb-2 border-b border-white/5 pb-2">
                                        <div className="flex justify-between text-[9px] text-gray-500">
                                            <span>[{new Date(trace.created_at).toLocaleTimeString()}] TRACE: {trace.langfuse_trace_id || trace.id}</span>
                                        </div>
                                        <div className="text-gray-300 whitespace-pre-wrap">
                                            {typeof trace.raw_trace === 'string' 
                                                ? trace.raw_trace 
                                                : JSON.stringify(trace.raw_trace, null, 2)}
                                        </div>
                                    </div>
                                ))
                            ) : (
                                <>
                                    <div className="text-gray-500">[0.00s] Initializing build123d kernel...</div>
                                    <div className="text-gray-500">[0.02s] Loading geometry policy: <span className="text-blue-400">STRICT_PHYSICS</span></div>
                                    <div className="text-blue-300">&gt;&gt;&gt; Running impl_build123d.py</div>
                                    <div className="text-green-500">[OK] BuildPart created. Vol: 12.4cm³</div>
                                    <div className="text-green-500">[OK] Vertex fillets applied. Max Curvature: 0.2</div>
                                    <div className="text-yellow-400">[WARN] Hole clearance near edge &lt; 2.0mm</div>
                                    <div className="text-gray-500 animate-pulse">_</div>
                                </>
                            )}
                        </div>
                        <div className="p-3 bg-panel-dark border-t border-border-dark">
                            <div className="flex items-center gap-3">
                                <div className="w-8 h-8 rounded bg-purple-500/10 border border-purple-500/30 flex items-center justify-center">
                                    <span className="material-symbols-outlined text-purple-400 text-lg">forum</span>
                                </div>
                                <div className="flex-1">
                                    <p className="text-[10px] font-bold text-purple-400">CRITIC: "Geometry is valid, but consider increasing fillet radius on the top flange to reduce stress concentrations identified in simulation."</p>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </aside>
        </div>

        {/* Footer */}
        <footer className="h-6 shrink-0 bg-[#070b10] border-t border-border-dark px-3 flex items-center justify-between text-[10px] text-text-secondary font-mono">
            <div className="flex items-center gap-4">
                <span className="flex items-center gap-1"><span className="w-2 h-2 rounded-full bg-green-500"></span> ENGINE: IDLE</span>
                <span>|</span>
                <span>LATENCY: 42ms</span>
            </div>
            <div className="flex items-center gap-4">
                <span>UTF-8</span>
                <span>Python 3.10</span>
                <span className="text-primary font-bold">build123d-v0.4</span>
            </div>
        </footer>
    </div>
  );
}
