export default function BenchmarkGeneration() {
  return (
    <div className="flex flex-col h-screen overflow-hidden bg-background-dark text-white font-display">
      {/* Header */}
      <header className="flex items-center justify-between border-b border-surface-border bg-surface-dark px-6 py-3 shrink-0 h-16">
        <div className="flex items-center gap-4">
          <div className="size-8 flex items-center justify-center bg-primary/20 rounded text-primary">
            <span className="material-symbols-outlined text-2xl">precision_manufacturing</span>
          </div>
          <div>
            <h2 className="text-white text-lg font-bold leading-tight tracking-tight">Benchmark Generation &amp; Validation</h2>
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
          <div className="flex gap-2">
            <button className="flex items-center justify-center rounded-lg h-9 w-9 bg-surface-border/30 text-white hover:bg-surface-border transition-colors">
              <span className="material-symbols-outlined text-xl">database</span>
            </button>
            <div className="h-9 w-9 rounded-full bg-primary flex items-center justify-center text-white font-bold text-sm border-2 border-surface-border">
              JD
            </div>
          </div>
        </div>
      </header>

      {/* Progress Steps */}
      <div className="border-b border-surface-border bg-[#0d161f] px-6 py-4 shrink-0">
        <div className="max-w-6xl mx-auto w-full">
          <div className="relative flex justify-between items-center w-full">
            <div className="absolute left-0 top-1/2 -translate-y-1/2 w-full h-0.5 bg-surface-border -z-0"></div>
            <div className="absolute left-0 top-1/2 -translate-y-1/2 w-2/3 h-0.5 bg-primary -z-0"></div>

            <div className="relative z-10 flex flex-col items-center gap-2">
              <div className="w-8 h-8 rounded-full bg-primary text-white flex items-center justify-center font-bold border-4 border-[#0d161f]">
                <span className="material-symbols-outlined text-xs">check</span>
              </div>
              <span className="text-primary text-[11px] font-bold uppercase tracking-wider">1. Intent</span>
            </div>
            <div className="relative z-10 flex flex-col items-center gap-2">
              <div className="w-8 h-8 rounded-full bg-primary text-white flex items-center justify-center font-bold border-4 border-[#0d161f]">
                <span className="material-symbols-outlined text-xs">check</span>
              </div>
              <span className="text-primary text-[11px] font-bold uppercase tracking-wider">2. Plan (Internal Review)</span>
            </div>
            <div className="relative z-10 flex flex-col items-center gap-2">
              <div className="w-10 h-10 rounded-full bg-primary text-white flex items-center justify-center shadow-[0_0_15px_rgba(30,148,246,0.4)] border-4 border-[#0d161f]">
                <span className="material-symbols-outlined text-lg">schema</span>
              </div>
              <span className="text-white text-[12px] font-black uppercase tracking-widest">3. CAD/XML Compilation</span>
            </div>
            <div className="relative z-10 flex flex-col items-center gap-2">
              <div className="w-8 h-8 rounded-full bg-surface-border text-text-secondary flex items-center justify-center font-bold border-4 border-[#0d161f]">
                4
              </div>
              <span className="text-text-secondary text-[11px] font-bold uppercase tracking-wider">4. Physics Validation</span>
            </div>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <main className="flex-1 flex overflow-hidden">
        {/* Left Pane */}
        <div className="w-5/12 flex flex-col border-r border-surface-border bg-surface-dark min-w-[450px]">
          <div className="flex-1 flex flex-col overflow-hidden">
            {/* Planner's Expected Solution */}
            <div className="h-1/3 flex flex-col border-b border-surface-border">
              <div className="px-4 py-2 bg-background-dark/40 border-b border-surface-border flex justify-between items-center">
                <h3 className="text-xs font-bold uppercase tracking-widest text-text-secondary flex items-center gap-2">
                  <span className="material-symbols-outlined text-sm text-primary">psychology</span>
                  Planner's Expected Solution
                </h3>
                <span className="text-[10px] text-primary/60 font-mono">v1.2.0-stable</span>
              </div>
              <div className="flex-1 p-4 overflow-y-auto custom-scrollbar bg-[#0a0f14]/30">
                <div className="space-y-3 font-mono text-[13px] text-gray-400">
                  <div className="bg-surface-border/10 p-2 border-l-2 border-primary">
                    <span className="text-primary-dark"># Logic:</span>
                    <p className="mt-1">Define agentive task where a pendulum must strike a hinged target with &gt; 2.5J kinetic energy.</p>
                  </div>
                  <div className="space-y-1 pl-2">
                    <p><span className="text-warning">GIVEN</span> platform_pos = [0, 0, 0]</p>
                    <p><span className="text-warning">WHEN</span> impulse applied to pivot_joint &gt;= 5.0</p>
                    <p><span className="text-warning">THEN</span> verify goal_zone.collision == true</p>
                  </div>
                </div>
              </div>
            </div>
            {/* Procedural CAD Script */}
            <div className="flex-1 flex flex-col">
              <div className="px-4 py-2 bg-background-dark/40 border-b border-surface-border flex justify-between items-center">
                <h3 className="text-xs font-bold uppercase tracking-widest text-text-secondary flex items-center gap-2">
                  <span className="material-symbols-outlined text-sm text-primary">terminal</span>
                  Procedural CAD Script (MuJoCo XML)
                </h3>
                <button className="text-[10px] flex items-center gap-1 hover:text-primary transition-colors uppercase font-bold text-text-secondary">
                  <span className="material-symbols-outlined text-xs">content_copy</span> Copy
                </button>
              </div>
              <div className="flex-1 bg-[#080c10] overflow-hidden flex flex-col font-mono text-sm group relative">
                <div className="flex-1 overflow-auto custom-scrollbar p-4 flex">
                  <div className="text-surface-border select-none text-right pr-4 border-r border-surface-border/20 mr-4 font-mono text-xs leading-6">
                    1<br/>2<br/>3<br/>4<br/>5<br/>6<br/>7<br/>8<br/>9<br/>10<br/>11<br/>12<br/>13<br/>14<br/>15<br/>16
                  </div>
                  <div className="text-gray-400 leading-6 whitespace-pre font-mono text-xs">
                    <span className="text-blue-400">&lt;mujoco&gt;</span>
                    {"\n"}
                    <span className="text-blue-400">&lt;worldbody&gt;</span>
                    {"\n"}
                    <span className="text-slate-500">&lt;!-- Procedural Benchmark Floor --&gt;</span>
                    {"\n"}
                    <span className="text-blue-400">&lt;geom</span> name="floor" pos="0 0 0" size="5 5 .1" type="plane"<span className="text-blue-400">/&gt;</span>
                    {"\n"}
                    <span className="text-blue-400">&lt;body</span> name="pendulum" pos="0 0 3"<span className="text-blue-400">&gt;</span>
                    {"\n"}
                    <span className="text-blue-400">&lt;joint</span> name="pivot" type="hinge" axis="0 1 0"<span className="text-blue-400">/&gt;</span>
                    {"\n"}
                    <span className="text-blue-400">&lt;geom</span> size="0.05 1.5" type="cylinder" mass="1"<span className="text-blue-400">/&gt;</span>
                    {"\n"}
                    <span className="text-blue-400">&lt;geom</span> pos="0 0 -1.5" size="0.2" type="sphere" rgba="0.8 0.2 0.2 1"<span className="text-blue-400">/&gt;</span>
                    {"\n"}
                    <span className="text-blue-400">&lt;/body&gt;</span>
                    {"\n"}
                    <span className="text-slate-500">&lt;!-- Target Region --&gt;</span>
                    {"\n"}
                    <span className="text-blue-400">&lt;site</span> name="goal" pos="1.5 0 0.5" size="0.3" rgba="0 1 0 0.3"<span className="text-blue-400">/&gt;</span>
                    {"\n"}
                    <span className="text-blue-400">&lt;/worldbody&gt;</span>
                    {"\n"}
                    <span className="text-blue-400">&lt;/mujoco&gt;</span>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Right Pane */}
        <div className="w-7/12 flex flex-col bg-background-dark">
          {/* 3D Viewport */}
          <div className="flex-1 relative viewport-gradient grid-bg overflow-hidden flex items-center justify-center group">
            <div className="absolute top-4 left-4 z-20 flex flex-col gap-2">
              <div className="bg-surface-dark/80 backdrop-blur border border-surface-border rounded-lg p-1 flex flex-col gap-1 shadow-2xl">
                <button className="p-2 hover:bg-primary/20 rounded-md text-white transition-colors">
                  <span className="material-symbols-outlined text-xl">view_in_ar</span>
                </button>
                <button className="p-2 hover:bg-primary/20 rounded-md text-text-secondary transition-colors">
                  <span className="material-symbols-outlined text-xl">videocam</span>
                </button>
                <div className="h-px bg-surface-border mx-1"></div>
                <button className="p-2 hover:bg-primary/20 rounded-md text-text-secondary transition-colors">
                  <span className="material-symbols-outlined text-xl">casino</span>
                </button>
              </div>
              <div className="px-2 py-1 bg-black/40 border border-surface-border/50 rounded text-[10px] text-text-secondary font-mono">
                SEED: 8841-A2
              </div>
            </div>
            <div className="absolute top-4 right-4 z-20 text-right space-y-2">
              <div className="bg-surface-dark/80 backdrop-blur border border-surface-border rounded-md px-3 py-1.5 shadow-lg flex items-center gap-3">
                <span className="text-[11px] font-bold uppercase tracking-wider text-text-secondary">Render Mode:</span>
                <span className="text-[11px] font-black text-primary">MUJOCO_NATIVE</span>
              </div>
              <div className="inline-block px-2 py-1 bg-warning/10 border border-warning/30 rounded text-[10px] text-warning font-bold">
                PREVIEWING RANDOMIZED VARIATION 3/10
              </div>
            </div>
            {/* 3D Object */}
            <div className="relative w-[500px] h-[350px]" style={{transformStyle: 'preserve-3d'}}>
              <div className="absolute bottom-10 left-1/2 -translate-x-1/2 w-[350px] h-[350px] bg-slate-800/40 border-2 border-slate-700 shadow-2xl" style={{transform: 'rotateX(65deg) rotateZ(45deg)'}}></div>
              <div className="absolute bottom-[180px] left-[55%] w-2 h-40 bg-slate-400/80" style={{transform: 'rotateX(65deg) rotateZ(45deg) translateY(-20px)'}}></div>
              <div className="absolute bottom-[120px] left-[45%] w-10 h-10 rounded-full bg-danger/80 shadow-[0_20px_40px_rgba(239,68,68,0.4)]" style={{transform: 'rotateX(65deg) rotateZ(45deg)'}}></div>
              <div className="absolute top-[80px] right-[100px] w-24 h-24 bg-success/10 border-2 border-dashed border-success/40 rounded-lg flex items-center justify-center" style={{transform: 'rotateX(65deg) rotateZ(45deg)'}}>
                <span className="text-success text-[10px] font-bold uppercase tracking-tighter opacity-80">TARGET_01</span>
              </div>
            </div>
          </div>

          {/* Integrity Checklist */}
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
                    <div className="h-1 w-full bg-surface-border rounded-full overflow-hidden">
                      <div className="h-full bg-success w-full"></div>
                    </div>
                    <p className="mt-2 text-[11px] text-gray-500 font-mono">MJCF Compliant (mj_v3.0)</p>
                  </div>
                  <div className="bg-background-dark/50 p-3 rounded-lg border border-surface-border">
                    <div className="flex items-center justify-between mb-2">
                      <span className="text-[10px] uppercase font-black text-text-secondary tracking-widest">Frame Stability (1s)</span>
                      <span className="material-symbols-outlined text-success text-lg">check_circle</span>
                    </div>
                    <div className="h-1 w-full bg-surface-border rounded-full overflow-hidden">
                      <div className="h-full bg-success w-full"></div>
                    </div>
                    <p className="mt-2 text-[11px] text-gray-500 font-mono">Max ΔE &lt; 1e-4</p>
                  </div>
                  <div className="bg-background-dark/50 p-3 rounded-lg border border-surface-border">
                    <div className="flex items-center justify-between mb-2">
                      <span className="text-[10px] uppercase font-black text-text-secondary tracking-widest">Intersection Check</span>
                      <span className="material-symbols-outlined text-primary text-lg animate-pulse">sync</span>
                    </div>
                    <div className="h-1 w-full bg-surface-border rounded-full overflow-hidden">
                      <div className="h-full bg-primary w-2/3"></div>
                    </div>
                    <p className="mt-2 text-[11px] text-gray-500 font-mono">Verifying contacts...</p>
                  </div>
                </div>
              </div>
              <div className="flex flex-col gap-3 self-end">
                <button className="px-6 py-3 rounded-xl bg-primary hover:bg-primary-dark text-white shadow-[0_0_20px_rgba(30,148,246,0.3)] transition-all font-bold flex items-center justify-center gap-3 group">
                  <span className="uppercase tracking-widest text-xs">Promote to Benchmark Suite</span>
                  <span className="material-symbols-outlined group-hover:translate-x-1 transition-transform">database_upload</span>
                </button>
                <div className="flex items-center justify-center gap-2">
                  <span className="w-1.5 h-1.5 rounded-full bg-slate-500"></span>
                  <span className="text-[9px] text-text-secondary uppercase font-bold tracking-tighter">Sync to shared_benchmarks.sqlite</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </div>
  );
}
