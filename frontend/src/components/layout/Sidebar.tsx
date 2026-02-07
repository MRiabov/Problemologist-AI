import { NavLink } from "react-router-dom";
import { LayoutDashboard, Rocket, Settings, Code, History } from "lucide-react";
import { cn } from "@/lib/utils";

const navigation = [
  { name: "Workspace", href: "/", icon: LayoutDashboard },
  { name: "Benchmark", href: "/benchmark", icon: Rocket },
  { name: "IDE", href: "/ide", icon: Code },
];

export default function Sidebar() {
  return (
    <div className="flex h-full w-64 flex-col border-r bg-card text-card-foreground">
      <div className="flex h-14 items-center border-b px-4">
        <div className="flex items-center gap-2 font-semibold">
          <div className="flex h-7 w-7 items-center justify-center rounded bg-primary text-primary-foreground">
            <span className="material-symbols-outlined text-xl">token</span>
          </div>
          <span className="tracking-tight">Agentic CAD</span>
        </div>
      </div>
      <div className="flex-1 overflow-y-auto py-4">
        <nav className="space-y-1 px-2">
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
      </div>
      <div className="border-t p-4">
        <nav className="space-y-1">
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
