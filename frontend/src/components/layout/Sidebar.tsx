import { NavLink, useLocation, useNavigate } from "react-router-dom";
import { LayoutDashboard, Rocket, Settings, History, CheckCircle2, XCircle, Clock, Search, Layers, Plus } from "lucide-react";
import { cn } from "../../lib/utils";
import { useEpisodes } from "../../context/EpisodeContext";
import { ScrollArea } from "../ui/scroll-area";
import { Input } from "../ui/input";
import { Button } from "../ui/button";
import { useState, useCallback } from "react";

const navigation = [
  { name: "Workspace", href: "/", icon: LayoutDashboard },
  { name: "Benchmark", href: "/benchmark", icon: Rocket },
];

export default function Sidebar() {
  const { episodes, selectedEpisode, selectEpisode, createNewBenchmark, loading } = useEpisodes();
  const location = useLocation();
  const navigate = useNavigate();
  const [filter, setFilter] = useState("");

  const filteredEpisodes = episodes.filter(ep => 
    ep.task.toLowerCase().includes(filter.toLowerCase()) || 
    ep.id.toLowerCase().includes(filter.toLowerCase())
  );

  const handleEpisodeClick = useCallback(async (id: string) => {
    await selectEpisode(id);

    const ep = episodes.find(e => e.id === id);
    if (ep && ep.metadata_vars?.episode_type === 'benchmark') {
      if (location.pathname !== '/benchmark' && location.pathname !== '/settings') {
        navigate('/benchmark');
      }
    } else if (ep) {
      if (location.pathname !== '/' && location.pathname !== '/settings') {
        navigate('/');
      }
    }

    if (location.pathname === '/settings') {
      // If we are in settings, we should go back to the appropriate UI.
      // For now, we go back to the previous page or default to workspace.
      navigate(-1);
    }
  }, [selectEpisode, episodes, location.pathname, navigate]);

  return (
    <div className="flex h-full w-full flex-col bg-card text-card-foreground">
      {/* App Header */}
      <div className="flex h-14 items-center border-b px-4 shrink-0">
        <div className="flex items-center gap-2 font-semibold">
          <div className="flex h-7 w-7 items-center justify-center rounded bg-primary text-primary-foreground">
            <Layers className="h-4 w-4" />
          </div>
          <span className="tracking-tight">Agentic CAD</span>
        </div>
      </div>

      {/* Main Navigation */}
      <div className="flex-1 overflow-hidden flex flex-col">
        <nav className="space-y-1 px-2 py-4 shrink-0">
          {navigation.map((item) => (
            <NavLink
              key={item.name}
              to={item.href}
              className={({ isActive }) =>
                cn(
                  "flex items-center gap-3 rounded-md px-3 py-2 text-sm font-medium transition-colors",
                  isActive
                    ? "bg-primary text-primary-foreground"
                    : "text-muted-foreground hover:bg-muted hover:text-foreground"
                )
              }
            >
              <item.icon className="h-4 w-4" />
              {item.name}
            </NavLink>
          ))}
        </nav>

        {/* Context-Aware History Section */}
        <div className="flex-1 flex flex-col min-h-0 border-t">
            <div className="p-4 space-y-3">
                <div className="flex items-center justify-between">
                    <h3 className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">
                        {(location.pathname === '/' || location.pathname === '/settings') ? 'Engineer History' : 'Benchmark Runs'}
                    </h3>
                    {(location.pathname === '/benchmark' || location.pathname === '/') ? (
                        <Button 
                            size="sm" 
                            className="h-6 px-2 text-[9px] font-bold gap-1"
                            onClick={() => createNewBenchmark(location.pathname === '/benchmark')}
                        >
                            <Plus className="h-3 w-3" />
                            CREATE NEW
                        </Button>
                    ) : (
                        <History className="h-3 w-3 text-muted-foreground" />
                    )}
                </div>
                <div className="relative">
                    <Search className="absolute left-2 top-2.5 h-3.5 w-3.5 text-muted-foreground" />
                    <Input 
                      className="h-8 pl-8 text-xs bg-muted/50" 
                      placeholder="Filter sessions..." 
                      value={filter}
                      onChange={(e) => setFilter(e.target.value)}
                    />
                </div>
            </div>
            <ScrollArea className="flex-1">
                <div className="p-2 space-y-1">
                    {loading ? (
                         <div className="p-4 text-xs text-muted-foreground text-center italic">Loading...</div>
                    ) : filteredEpisodes.length === 0 ? (
                         <div className="p-4 text-xs text-muted-foreground text-center">No history found.</div>
                    ) : (
                        filteredEpisodes.map(ep => (
                            <button 
                                key={ep.id} 
                                onClick={() => handleEpisodeClick(ep.id)}
                                className={cn(
                                  "w-full text-left p-2.5 rounded-md transition-all group border border-transparent overflow-hidden",
                                  selectedEpisode?.id === ep.id 
                                    ? "bg-primary/10 border-primary/20" 
                                    : "hover:bg-muted/50"
                                )}
                            >
                                <div className="flex justify-between items-start mb-1 min-w-0">
                                    <span className={cn(
                                      "text-[11px] font-semibold truncate flex-1 pr-2",
                                      selectedEpisode?.id === ep.id ? "text-primary" : "text-foreground"
                                    )}>
                                      {ep.task || ep.id.substring(0,8)}
                                    </span>
                                    {ep.status === 'running' ? (
                                        <div className="h-1.5 w-1.5 rounded-full bg-primary animate-pulse mt-1" />
                                    ) : ep.status === 'completed' ? (
                                        <CheckCircle2 className="h-3 w-3 text-green-500" />
                                    ) : (
                                        <XCircle className="h-3 w-3 text-destructive" />
                                    )}
                                </div>
                                <div className="flex items-center gap-2 text-[9px] text-muted-foreground">
                                    <Clock className="h-2.5 w-2.5" />
                                    <span>{new Date(ep.created_at).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}</span>
                                    <span>•</span>
                                    <span className="font-mono">{ep.id.substring(0,4)}</span>
                                </div>
                            </button>
                        ))
                    )}
                </div>
            </ScrollArea>
        </div>
      </div>

      {/* Bottom Settings Navigation */}
      <div className="border-t p-4 shrink-0">
        <nav className="space-y-1">
          {/* 
          <NavLink
            to="/history"
            className={({ isActive }) =>
              cn(
                "flex items-center gap-3 rounded-md px-3 py-2 text-sm font-medium transition-colors",
                isActive
                  ? "bg-primary text-primary-foreground"
                  : "text-muted-foreground hover:bg-muted hover:text-foreground"
              )
            }
          >
            <History className="h-4 w-4" />
            History
          </NavLink>
          */}
          <NavLink
            to="/settings"
            className={({ isActive }) =>
              cn(
                "flex items-center gap-3 rounded-md px-3 py-2 text-sm font-medium transition-colors",
                isActive
                  ? "bg-primary text-primary-foreground"
                  : "text-muted-foreground hover:bg-muted hover:text-foreground"
              )
            }
          >
            <Settings className="h-4 w-4" />
            Settings
          </NavLink>
        </nav>
      </div>
    </div>
  );
}
