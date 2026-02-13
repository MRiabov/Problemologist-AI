import { Outlet } from "react-router-dom";
import Sidebar from "./Sidebar";
import { useConnection } from "../../context/ConnectionContext";
import { AlertTriangle } from "lucide-react";

export default function AppLayout() {
  const { isMockMode } = useConnection();

  return (
    <div className="grid grid-cols-12 h-screen w-full overflow-hidden bg-background text-foreground">
      <div className="col-span-3 h-full overflow-hidden border-r border-border">
        <Sidebar />
      </div>
      <main className="col-span-9 flex flex-col min-w-0 overflow-hidden relative">
        {isMockMode && (
          <div className="bg-amber-500/10 border-b border-amber-500/20 px-4 py-2 flex items-center gap-3 z-50 animate-in slide-in-from-top duration-300">
            <AlertTriangle className="h-4 w-4 text-amber-500" />
            <div className="flex flex-col">
              <span className="text-[10px] font-black uppercase tracking-wider text-amber-500">Mock LLM Mode Active</span>
              <span className="text-[10px] text-amber-500/70">Real reasoning traces are disabled. Set IS_INTEGRATION_TEST=false to use real LLM.</span>
            </div>
          </div>
        )}
        <Outlet />
      </main>
    </div>
  );
}
