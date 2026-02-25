import { X, Code, Box, Cpu } from "lucide-react";
import { Badge } from "../ui/badge";
import { Button } from "../ui/button";
import { useEpisodes, type ContextItem } from "../../context/EpisodeContext";

const ContextIcon = ({ type }: { type: ContextItem['type'] }) => {
  switch (type) {
    case 'code': return <Code className="h-3 w-3" />;
    case 'cad': return <Box className="h-3 w-3" />;
    case 'circuit': return <Cpu className="h-3 w-3" />;
    default: return null;
  }
};

export function ContextCards() {
  const { selectedContext, removeFromContext, clearContext } = useEpisodes();

  if (selectedContext.length === 0) return null;

  return (
    <div className="flex flex-wrap items-center gap-2 p-2 bg-muted/30 border-t border-border/50 animate-in slide-in-from-bottom-2 fade-in">
      <div className="flex items-center gap-1.5 mr-2">
         <span className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">Context</span>
         <Badge variant="secondary" className="h-4 px-1 text-[9px] font-bold">
            {selectedContext.length}
         </Badge>
      </div>
      
      <div className="flex flex-wrap gap-2 flex-1">
        {selectedContext.map((item) => (
          <div 
            key={item.id}
            data-testid="context-card"
            className="group flex items-center gap-2 px-2 py-1 bg-background border border-border/50 rounded-md shadow-sm hover:border-primary/50 transition-colors"
          >
            <div className="text-primary/70">
                <ContextIcon type={item.type} />
            </div>
            <span className="text-[11px] font-medium max-w-[120px] truncate">
                {item.label}
            </span>
            <button 
              onClick={() => removeFromContext(item.id)}
              className="p-0.5 hover:bg-muted rounded text-muted-foreground hover:text-destructive transition-colors"
            >
              <X className="h-3 w-3" />
            </button>
          </div>
        ))}
      </div>

      <Button 
        variant="ghost" 
        size="sm" 
        onClick={clearContext}
        className="h-7 px-2 text-[10px] text-muted-foreground hover:text-destructive"
      >
        Clear All
      </Button>
    </div>
  );
}
