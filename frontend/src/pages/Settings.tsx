import { useTheme } from "../context/ThemeContext";
import { Moon, Sun, Settings as SettingsIcon, ChevronLeft } from "lucide-react";
import { cn } from "../lib/utils";
import { useNavigate } from "react-router-dom";
import { Button } from "../components/ui/button";
import { useUISettings } from "../context/UISettingsContext";

export default function Settings() {
  const { theme, setTheme } = useTheme();
  const { viewReasoning, setViewReasoning } = useUISettings();
  const navigate = useNavigate();

  const themeOptions = [
    { id: 'light', name: 'Light', icon: Sun },
    { id: 'dark', name: 'Dark', icon: Moon },
  ];

  return (
    <div className="flex-1 p-8 bg-background overflow-auto">
      <div className="max-w-2xl mx-auto space-y-8">
        <div className="flex items-center justify-between border-b pb-4">
          <div className="flex items-center gap-3">
            <SettingsIcon className="h-6 w-6 text-primary" />
            <h1 className="text-2xl font-bold tracking-tight">Settings</h1>
          </div>
          <Button 
            variant="ghost" 
            size="sm" 
            className="gap-2 text-muted-foreground hover:text-foreground"
            onClick={() => navigate(-1)}
          >
            <ChevronLeft className="h-4 w-4" />
            Back to UI
          </Button>
        </div>

        <section className="space-y-4">
          <div>
            <h2 className="text-lg font-semibold">Appearance</h2>
            <p className="text-sm text-muted-foreground">Customize how the application looks.</p>
          </div>

          <div className="grid grid-cols-2 gap-4">
            {themeOptions.map((option) => (
              <button
                key={option.id}
                onClick={() => setTheme(option.id as 'light' | 'dark')}
                className={cn(
                  "flex flex-col items-center gap-3 p-4 rounded-xl border-2 transition-all",
                  theme === option.id
                    ? "border-primary bg-primary/5"
                    : "border-border hover:border-muted-foreground/30 bg-card"
                )}
              >
                <option.icon className={cn(
                  "h-8 w-8",
                  theme === option.id ? "text-primary" : "text-muted-foreground"
                )} />
                <span className={cn(
                  "text-sm font-medium",
                  theme === option.id ? "text-primary" : "text-muted-foreground"
                )}>{option.name}</span>
              </button>
            ))}
          </div>
        </section>

        <section className="space-y-4">
          <div>
            <h2 className="text-lg font-semibold">Workspace</h2>
            <p className="text-sm text-muted-foreground">Control trace visibility in chat.</p>
          </div>

          <button
            data-testid="settings-view-reasoning-toggle"
            onClick={() => setViewReasoning(!viewReasoning)}
            className={cn(
              "w-full flex items-center justify-between p-4 rounded-xl border-2 transition-all text-left",
              viewReasoning
                ? "border-primary bg-primary/5"
                : "border-border hover:border-muted-foreground/30 bg-card"
            )}
          >
            <div>
              <div className="text-sm font-medium">View reasoning</div>
              <div className="text-xs text-muted-foreground">
                Show collapsible reasoning spans between steps.
              </div>
            </div>
            <span
              className={cn(
                "text-xs font-semibold px-2 py-1 rounded-md",
                viewReasoning
                  ? "bg-primary/20 text-primary"
                  : "bg-muted text-muted-foreground"
              )}
            >
              {viewReasoning ? "On" : "Off"}
            </span>
          </button>
        </section>
      </div>
    </div>
  );
}
