import { useState } from "react";
import { ChevronRight, ChevronDown } from "lucide-react";
import { cn } from "../../lib/utils";

interface ThoughtBlockProps {
  duration?: number;
  content?: string;
  className?: string;
}

export function ThoughtBlock({ duration = 0, content, className }: ThoughtBlockProps) {
  const [isOpen, setIsOpen] = useState(false);

  if (!content && duration === 0) return null;

  return (
    <div className={cn("space-y-1 mb-2", className)}>
      <button 
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center gap-1.5 text-[12px] text-muted-foreground/60 hover:text-muted-foreground transition-colors py-0.5"
      >
        {isOpen ? <ChevronDown className="h-3 w-3" /> : <ChevronRight className="h-3 w-3" />}
        <span className="font-medium tracking-tight">Thought for {duration}s</span>
      </button>
      
      {isOpen && content && (
        <div className="text-muted-foreground/80 whitespace-pre-wrap text-[13px] leading-relaxed pl-4 border-l border-border/30 ml-1.5 py-1 animate-in fade-in slide-in-from-left-1 duration-200">
          {content}
        </div>
      )}
    </div>
  );
}
