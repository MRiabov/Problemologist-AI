export default function IdeDashboard() {
  return (
    <div className="flex flex-col h-screen overflow-hidden bg-background-dark text-white font-display">
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
            <input className="bg-transparent border-none text-sm w-full text-white placeholder-text-secondary focus:ring-0 focus:outline-none" placeholder="Search files, symbols, or agents..." type="text"/>
            <div className="flex gap-1">
              <kbd className="hidden sm:inline-block px-1.5 py-0.5 text-xs text-text-secondary bg-border-dark rounded border border-[#304d63]">⌘K</kbd>
            </div>
          </div>
        </div>
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-4 mr-4">
            <a className="text-text-secondary hover:text-white text-sm font-medium transition-colors" href="#">File</a>
            <a className="text-text-secondary hover:text-white text-sm font-medium transition-colors" href="#">View</a>
            <a className="text-text-secondary hover:text-white text-sm font-medium transition-colors" href="#">Help</a>
          </div>
          <button className="flex items-center justify-center gap-2 rounded-md h-8 px-4 bg-primary hover:bg-blue-600 transition-colors text-white text-xs font-bold uppercase tracking-wider">
            <span className="material-symbols-outlined text-base">play_arrow</span>
            Run
          </button>
          <div className="w-8 h-8 rounded-full bg-gradient-to-tr from-purple-500 to-blue-500 border border-white/10" title="User profile avatar gradient"></div>
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
            {/* Search Input */}
            <div className="relative">
              <span className="absolute left-2 top-1/2 -translate-y-1/2 material-symbols-outlined text-text-secondary text-lg">search</span>
              <input className="w-full bg-panel-dark border border-border-dark rounded-md py-1.5 pl-8 pr-3 text-sm text-white focus:outline-none focus:border-primary" placeholder="Filter episodes..." type="text"/>
            </div>
            {/* Chips */}
            <div className="flex flex-wrap gap-2">
              <span className="px-2 py-0.5 rounded text-xs font-medium bg-primary/20 text-primary border border-primary/30 cursor-pointer">All</span>
              <span className="px-2 py-0.5 rounded text-xs font-medium bg-panel-dark text-text-secondary border border-border-dark hover:border-text-secondary cursor-pointer">#Kinematic</span>
              <span className="px-2 py-0.5 rounded text-xs font-medium bg-panel-dark text-text-secondary border border-border-dark hover:border-text-secondary cursor-pointer">#Spatial</span>
            </div>
          </div>
          {/* List Items */}
          <div className="flex-1 overflow-y-auto">
            {/* Active Item */}
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
            {/* History Items */}
            <div className="group flex flex-col p-3 border-l-2 border-transparent hover:bg-panel-dark cursor-pointer transition-colors border-b border-border-dark/50">
              <div className="flex justify-between items-start mb-1">
                <span className="text-sm font-medium text-text-secondary group-hover:text-white">Ep-1024: Bracket Opt</span>
                <span className="material-symbols-outlined text-green-500 text-sm">check_circle</span>
              </div>
              <div className="flex items-center justify-between text-xs text-text-secondary">
                <span>12m ago</span>
                <div className="flex gap-1">
                  <span className="text-[10px] bg-border-dark px-1 rounded">#Spatial</span>
                </div>
              </div>
            </div>
            {/* ... more items ... */}
          </div>
        </aside>

        {/* Middle Pane: Split Code & Agent Cognition */}
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
          {/* Code Editor Area */}
          <div className="flex-1 relative bg-code-bg font-mono text-sm overflow-auto">
            <div className="flex min-h-full">
              {/* Line Numbers */}
              <div className="w-12 flex-shrink-0 bg-[#0d1116] text-gray-600 text-right pr-3 pt-4 select-none border-r border-border-dark/30 leading-6 text-xs">
                1<br/>2<br/>3<br/>4<br/>5<br/>6<br/>7<br/>8<br/>9<br/>10<br/>11<br/>12<br/>13<br/>14
              </div>
              {/* Code Content */}
              <div className="flex-1 pl-4 pt-4 text-gray-300 leading-6 text-xs">
                <div><span className="syntax-keyword">import</span> cad_library <span className="syntax-keyword">as</span> cad</div>
                <div><span className="syntax-keyword">from</span> agents.physics <span className="syntax-keyword">import</span> ConstraintSolver</div>
                <br/>
                <div><span className="syntax-comment"># Initialize the main assembly context</span></div>
                <div><span className="syntax-keyword">def</span> <span className="syntax-func">optimize_bracket_structure</span>(params):</div>
                <div className="pl-4">solver = <span className="syntax-class">ConstraintSolver</span>(tolerance=1e-5)</div>
                <div className="pl-4">base_plate = cad.<span className="syntax-func">create_box</span>(width=params[<span className="syntax-string">'w'</span>], height=10, depth=50)</div>
                <br/>
                <div className="pl-4"><span className="syntax-comment"># Agent generated adjustment based on interference</span></div>
                <div className="pl-4 relative bg-primary/10 -mx-4 px-4 border-l-2 border-primary">
                  <span className="absolute right-2 top-0 text-[10px] text-primary opacity-50">AGENT EDITING</span>
                  if solver.<span className="syntax-func">check_collision</span>(base_plate):
                </div>
                <div className="pl-4 relative bg-primary/10 -mx-4 px-4 border-l-2 border-primary">
                  &nbsp;&nbsp;&nbsp;&nbsp;params[<span className="syntax-string">'w'</span>] += 2.5 <span className="syntax-comment"># Expanding width to resolve conflict</span>
                </div>
                <div className="pl-4">
                  return base_plate
                </div>
              </div>
            </div>
          </div>
          {/* Agent Cognition / Chain of Thought Panel */}
          <div className="h-[35%] border-t border-border-dark flex flex-col bg-[#11161d]">
            <div className="px-3 py-2 border-b border-border-dark flex justify-between items-center bg-[#16202a]">
              <div className="flex items-center gap-2">
                <span className="material-symbols-outlined text-purple-400 text-lg">psychology</span>
                <span className="text-xs font-bold uppercase tracking-wider text-text-secondary">Agent Cognition Stream</span>
              </div>
              <span className="px-2 py-0.5 rounded-full bg-green-500/20 text-green-400 text-[10px] border border-green-500/30">Thinking</span>
            </div>
            <div className="flex-1 overflow-y-auto p-4 space-y-4">
              {/* Past Thought */}
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
              {/* Current Thought */}
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
              {/* Action */}
              <div className="flex gap-3 animate-pulse">
                <div className="w-6 flex-shrink-0 flex flex-col items-center">
                  <div className="w-1 h-2 bg-border-dark/50 mb-1"></div>
                  <div className="w-6 h-6 rounded-full bg-primary/20 flex items-center justify-center border border-primary/50">
                    <span className="material-symbols-outlined text-xs text-primary">terminal</span>
                  </div>
                </div>
                <div className="flex-1 mt-3">
                  <p className="text-xs text-primary mb-1">Executing</p>
                  <span className="text-xs font-mono text-gray-500">Writing to generator.py line 8...</span>
                </div>
              </div>
            </div>
          </div>
        </main>
        {/* Right Pane: 3D Viewport & Terminal */}
        <aside className="w-[420px] flex flex-col border-l border-border-dark bg-[#0f151b]">
          {/* 3D Viewport Area */}
          <div className="h-[60%] relative bg-gradient-to-br from-[#1a2634] to-[#0f151b] flex items-center justify-center overflow-hidden">
            {/* Toolbar */}
            <div className="absolute top-4 left-4 z-10 flex flex-col gap-2 bg-black/40 backdrop-blur-sm p-1 rounded-md border border-white/5">
              <button className="w-8 h-8 flex items-center justify-center rounded hover:bg-white/10 text-white/80" title="Orbit">
                <span className="material-symbols-outlined text-lg">3d_rotation</span>
              </button>
              <button className="w-8 h-8 flex items-center justify-center rounded hover:bg-white/10 text-white/80" title="Pan">
                <span className="material-symbols-outlined text-lg">pan_tool</span>
              </button>
              <button className="w-8 h-8 flex items-center justify-center rounded hover:bg-white/10 text-white/80" title="Zoom">
                <span className="material-symbols-outlined text-lg">zoom_in</span>
              </button>
              <div className="h-px w-full bg-white/10 my-0.5"></div>
              <button className="w-8 h-8 flex items-center justify-center rounded hover:bg-white/10 text-white/80" title="Wireframe">
                <span className="material-symbols-outlined text-lg">grid_4x4</span>
              </button>
            </div>
            {/* Floating Metrics Overlay */}
            <div className="absolute top-4 right-4 z-10 w-48 bg-black/60 backdrop-blur-md rounded-lg border border-border-dark shadow-xl overflow-hidden">
              <div className="px-3 py-2 border-b border-white/5 bg-white/5">
                <h4 className="text-xs font-bold text-white uppercase">Live Metrics</h4>
              </div>
              <div className="p-3 space-y-3">
                <div className="flex justify-between items-center">
                  <span className="text-xs text-text-secondary flex items-center gap-1">
                    <span className="material-symbols-outlined text-yellow-400 text-[14px]">bolt</span>
                    Energy
                  </span>
                  <span className="text-sm font-mono text-white">420kJ</span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-xs text-text-secondary flex items-center gap-1">
                    <span className="material-symbols-outlined text-green-400 text-[14px]">attach_money</span>
                    Cost
                  </span>
                  <span className="text-sm font-mono text-white">$0.04</span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-xs text-text-secondary flex items-center gap-1">
                    <span className="material-symbols-outlined text-red-400 text-[14px]">mobile_text</span>
                    Impulses
                  </span>
                  <span className="text-sm font-mono text-white">0</span>
                </div>
              </div>
            </div>
            {/* 3D Content Placeholder */}
            <div className="w-full h-full relative">
              {/* Abstract representation of 3D object using CSS shapes/gradients */}
              <div className="absolute inset-0 flex items-center justify-center">
                <div className="relative w-48 h-48 border-4 border-blue-500/30 rounded-full flex items-center justify-center animate-[spin_10s_linear_infinite]">
                  <div className="w-32 h-32 border-4 border-purple-500/40 rounded-full"></div>
                  <div className="absolute w-full h-1 bg-blue-500/20 top-1/2 left-0 -translate-y-1/2"></div>
                  <div className="absolute h-full w-1 bg-blue-500/20 left-1/2 top-0 -translate-x-1/2"></div>
                </div>
                {/* Highlighted "Part" */}
                <div className="absolute w-24 h-24 bg-primary/10 border border-primary rounded shadow-[0_0_30px_rgba(30,148,246,0.3)] backdrop-blur-sm flex items-center justify-center">
                  <div className="text-[10px] text-primary font-mono tracking-widest">BRACKET</div>
                </div>
              </div>
              {/* Grid Floor */}
              <div className="absolute bottom-0 w-full h-1/2 bg-[linear-gradient(to_bottom,transparent_0%,rgba(30,148,246,0.1)_100%)]">
                <div className="w-full h-full" style={{backgroundImage: 'linear-gradient(rgba(255, 255, 255, 0.05) 1px, transparent 1px), linear-gradient(90deg, rgba(255, 255, 255, 0.05) 1px, transparent 1px)', backgroundSize: '20px 20px', perspective: '100px', transform: 'rotateX(60deg) scale(2)'}}></div>
              </div>
            </div>
          </div>
          {/* Execution Log Terminal */}
          <div className="flex-1 border-t border-border-dark bg-[#0a0e12] flex flex-col min-h-0">
            <div className="px-3 py-1.5 border-b border-border-dark flex justify-between items-center bg-[#16202a]">
              <div className="flex items-center gap-2">
                <span className="material-symbols-outlined text-text-secondary text-sm">terminal</span>
                <span className="text-xs font-bold uppercase tracking-wider text-text-secondary">Execution Log</span>
              </div>
              <div className="flex gap-2">
                <button className="text-[10px] text-text-secondary hover:text-white uppercase">Clear</button>
                <button className="text-[10px] text-text-secondary hover:text-white uppercase">Filter</button>
              </div>
            </div>
            <div className="flex-1 p-3 font-mono text-xs overflow-y-auto space-y-1">
              <div className="text-gray-500">
                <span className="text-blue-500">[INFO]</span> 14:02:22 Simulation environment initialized.
              </div>
              <div className="text-gray-500">
                <span className="text-blue-500">[INFO]</span> 14:02:23 Loading physics engine (PhysX 5.1)...
              </div>
              <div className="text-gray-400">
                <span className="text-purple-400">[AGENT]</span> 14:02:24 Starting geometry generation loop.
              </div>
              <div className="text-gray-300">
                &gt; Generating mesh for 'bracket_v2'... Done (45ms)
              </div>
              <div className="text-yellow-500/90 bg-yellow-500/10 p-1 rounded border border-yellow-500/20">
                <span className="font-bold">[WARN]</span> 14:02:25 Mesh intersection detected at vertex #402. Tolerance exceeded (0.02mm &gt; 0.01mm).
              </div>
              <div className="text-gray-400">
                <span className="text-purple-400">[AGENT]</span> 14:02:25 Intercepted warning. Applying corrective transformation.
              </div>
              <div className="text-gray-300">
                &gt; Re-calculating constraints...
              </div>
              <div className="text-green-500/90">
                <span className="font-bold">[SUCCESS]</span> 14:02:26 Optimization complete. Energy reduced by 4%.
              </div>
              <div className="text-gray-500 animate-pulse">_</div>
            </div>
          </div>
        </aside>
      </div>
    </div>
  );
}
