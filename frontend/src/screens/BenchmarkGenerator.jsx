import React, { useState } from 'react';
import { Link } from 'react-router-dom';
import client from '../api/client';

const BenchmarkGenerator = () => {
    const [stage, setStage] = useState('INPUT'); // INPUT, PLAN, CODE, FINAL
    const [intent, setIntent] = useState('');
    const [plan, setPlan] = useState('');
    const [generatedData, setGeneratedData] = useState({ code: '', mjcf: '', renders: [], logs: [] });
    const [loading, setLoading] = useState(false);

    const handleGeneratePlan = async () => {
        setLoading(true);
        try {
            const res = await client.post('/benchmark/plan', { intent });
            setPlan(res.data.plan);
            setStage('PLAN');
        } catch (e) {
            console.error(e);
            alert("Planning failed");
        }
        setLoading(false);
    };

    const handleGenerateCad = async () => {
        setLoading(true);
        try {
            const res = await client.post('/benchmark/generate', { intent, plan });
            setGeneratedData(res.data);
            setStage('CODE');
        } catch (e) {
            console.error(e);
            alert("Generation failed");
        }
        setLoading(false);
    };

    return (
        <div className="bg-background-dark text-white h-screen flex flex-col overflow-hidden">
            {/* Header */}
            <header className="flex items-center justify-between border-b border-surface-border bg-surface-dark px-6 py-3 shrink-0 h-16">
                <div className="flex items-center gap-4">
                     <Link to="/" className="size-8 flex items-center justify-center bg-primary/20 rounded text-primary hover:bg-primary/30 transition-colors">
                        <span className="material-symbols-outlined text-2xl">arrow_back</span>
                    </Link>
                    <div>
                        <h2 className="text-white text-lg font-bold leading-tight tracking-tight">Benchmark Generation & Validation</h2>
                        <p className="text-[10px] text-text-secondary uppercase tracking-widest font-bold">Mechanical Physics Pipeline</p>
                    </div>
                </div>
                 <div className="flex items-center gap-6">
                    <div className="flex items-center gap-3 px-3 py-1.5 bg-background-dark/80 border border-surface-border rounded-full shadow-inner">
                        <div className="relative flex items-center justify-center">
                            <span className="absolute inline-flex h-2 w-2 rounded-full bg-success opacity-75"></span>
                            <span className="relative inline-flex rounded-full h-2 w-2 bg-success"></span>
                        </div>
                        <span className="text-[11px] font-bold text-success uppercase tracking-wider">Isolated Container Active</span>
                    </div>
                 </div>
            </header>

            {/* Progress Bar */}
            <div className="border-b border-surface-border bg-[#0d161f] px-6 py-4 shrink-0">
                <div className="max-w-6xl mx-auto w-full relative flex justify-between items-center">
                    <div className="absolute left-0 top-1/2 -translate-y-1/2 w-full h-0.5 bg-surface-border -z-0"></div>
                     {/* Dynamic Progress Bar */}
                    <div className="absolute left-0 top-1/2 -translate-y-1/2 h-0.5 bg-primary -z-0 transition-all duration-500"
                         style={{width: stage === 'INPUT' ? '0%' : stage === 'PLAN' ? '33%' : stage === 'CODE' ? '66%' : '100%'}}></div>

                    {['Intent', 'Plan', 'CAD/XML', 'Validation'].map((label, idx) => {
                         const isActive = (stage === 'INPUT' && idx === 0) || (stage === 'PLAN' && idx <= 1) || (stage === 'CODE' && idx <= 2) || (stage === 'FINAL');
                         return (
                            <div key={label} className="relative z-10 flex flex-col items-center gap-2">
                                <div className={`w-8 h-8 rounded-full flex items-center justify-center font-bold border-4 border-[#0d161f] transition-colors ${isActive ? 'bg-primary text-white' : 'bg-surface-border text-text-secondary'}`}>
                                    {idx + 1}
                                </div>
                                <span className={`text-[11px] font-bold uppercase tracking-wider ${isActive ? 'text-primary' : 'text-text-secondary'}`}>{label}</span>
                            </div>
                         );
                    })}
                </div>
            </div>

            {/* Main Content */}
            <main className="flex-1 flex overflow-hidden">
                {/* Left Pane */}
                <div className="w-5/12 flex flex-col border-r border-surface-border bg-surface-dark min-w-[450px]">
                     {stage === 'INPUT' && (
                         <div className="p-8 flex flex-col gap-4">
                             <h3 className="text-xl font-bold">Define Benchmark Intent</h3>
                             <textarea
                                className="bg-background-dark border border-surface-border rounded p-4 text-white h-48 focus:border-primary outline-none"
                                placeholder="Describe the physics scenario..."
                                value={intent}
                                onChange={(e) => setIntent(e.target.value)}
                            />
                             <button
                                onClick={handleGeneratePlan}
                                disabled={loading || !intent}
                                className="bg-primary hover:bg-primary-dark text-white font-bold py-2 px-4 rounded disabled:opacity-50 flex items-center justify-center gap-2">
                                    {loading ? 'Generating...' : 'Generate Plan'}
                                    <span className="material-symbols-outlined">arrow_forward</span>
                             </button>
                         </div>
                     )}

                     {(stage === 'PLAN' || stage === 'CODE' || stage === 'FINAL') && (
                        <div className="flex-1 flex flex-col overflow-hidden">
                            <div className="h-1/3 flex flex-col border-b border-surface-border">
                                <div className="px-4 py-2 bg-background-dark/40 border-b border-surface-border flex justify-between items-center">
                                    <h3 className="text-xs font-bold uppercase tracking-widest text-text-secondary flex items-center gap-2">
                                        <span className="material-symbols-outlined text-sm text-primary">psychology</span>
                                        Planner's Expected Solution
                                    </h3>
                                </div>
                                <div className="flex-1 p-4 overflow-y-auto bg-[#0a0f14]/30 font-mono text-sm text-gray-300">
                                    {plan}
                                    {stage === 'PLAN' && (
                                        <button
                                            onClick={handleGenerateCad}
                                            disabled={loading}
                                            className="mt-4 bg-primary hover:bg-primary-dark text-white font-bold py-2 px-4 rounded w-full flex items-center justify-center gap-2">
                                            {loading ? 'Compiling CAD...' : 'Generate CAD & XML'}
                                            <span className="material-symbols-outlined">precision_manufacturing</span>
                                        </button>
                                    )}
                                </div>
                            </div>

                            {(stage === 'CODE' || stage === 'FINAL') && (
                                <div className="flex-1 flex flex-col">
                                    <div className="px-4 py-2 bg-background-dark/40 border-b border-surface-border flex justify-between items-center">
                                        <h3 className="text-xs font-bold uppercase tracking-widest text-text-secondary flex items-center gap-2">
                                            <span className="material-symbols-outlined text-sm text-primary">terminal</span>
                                            Procedural CAD Script (MuJoCo XML)
                                        </h3>
                                    </div>
                                    <div className="flex-1 bg-[#080c10] overflow-auto p-4 font-mono text-xs text-gray-400 whitespace-pre">
                                        {generatedData.mjcf || generatedData.code || "Generating..."}
                                    </div>
                                </div>
                            )}
                        </div>
                     )}
                </div>

                {/* Right Pane (Viewport) */}
                <div className="w-7/12 flex flex-col bg-background-dark">
                    <div className="flex-1 relative flex items-center justify-center group bg-[#1a2a3a]">
                         {loading ? (
                             <div className="flex flex-col items-center gap-4">
                                 <div className="w-12 h-12 border-4 border-primary border-t-transparent rounded-full animate-spin"></div>
                                 <span className="text-primary font-bold animate-pulse">Processing...</span>
                             </div>
                         ) : (
                             generatedData.renders && generatedData.renders.length > 0 ? (
                                 <img src={generatedData.renders[0]} alt="Render" className="max-w-full max-h-full object-contain" />
                             ) : (
                                 <div className="text-white opacity-20 text-4xl font-black">VIEWPORT</div>
                             )
                         )}
                    </div>
                    {/* Checklist */}
                    {(stage === 'CODE' || stage === 'FINAL') && !loading && (
                        <div className="bg-surface-dark border-t border-surface-border p-5">
                            <div className="flex items-start justify-between">
                                <div className="space-y-4 flex-1">
                                    <div className="flex items-center gap-2">
                                        <span className="material-symbols-outlined text-primary text-xl">verified_user</span>
                                        <h4 className="text-sm font-bold uppercase tracking-[0.15em] text-white">MuJoCo Integrity Checklist</h4>
                                    </div>
                                    <div className="grid grid-cols-3 gap-6 max-w-4xl">
                                         <div className="bg-background-dark/50 p-3 rounded-lg border border-surface-border">
                                            <div className="flex items-center justify-between mb-2">
                                                <span className="text-[10px] uppercase font-black text-text-secondary tracking-widest">XML Schema Validation</span>
                                                <span className="material-symbols-outlined text-success text-lg">check_circle</span>
                                            </div>
                                            <div className="h-1 w-full bg-surface-border rounded-full overflow-hidden"><div className="h-full bg-success w-full"></div></div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    )}
                </div>
            </main>
        </div>
    );
};

export default BenchmarkGenerator;
