import { WifiOff, AlertCircle } from "lucide-react";
import { cn } from "../../lib/utils";

interface ConnectionErrorProps {
  className?: string;
  title?: string;
  message?: string;
}

export default function ConnectionError({ 
  className, 
  title = "No Connection", 
  message = "Unable to reach the benchmark server" 
}: ConnectionErrorProps) {
  return (
    <div className={cn(
      "flex flex-col items-center justify-center h-full w-full bg-background/50 backdrop-blur-[2px] z-50 p-6 text-center animate-in fade-in duration-500",
      className
    )}>
      <div className="relative mb-4">
        <div className="absolute inset-0 bg-red-500/20 blur-2xl rounded-full animate-pulse" />
        <div className="relative size-16 flex items-center justify-center rounded-2xl bg-red-500/10 border border-red-500/20 text-red-500">
            <WifiOff className="h-8 w-8" />
        </div>
        <div className="absolute -top-1 -right-1">
            <div className="relative">
                <div className="absolute inset-0 bg-red-500 animate-ping rounded-full opacity-75" />
                <AlertCircle className="relative h-4 w-4 text-red-500 fill-background" />
            </div>
        </div>
      </div>
      
      <h3 className="text-sm font-bold tracking-tight uppercase mb-1">{title}</h3>
      <p className="text-[11px] text-muted-foreground max-w-[180px] leading-relaxed">
        {message}
      </p>
      
      <div className="mt-6 flex items-center gap-2">
        <div className="h-[1px] w-8 bg-border" />
        <span className="text-[9px] font-black uppercase tracking-[0.2em] text-muted-foreground/50">Offline</span>
        <div className="h-[1px] w-8 bg-border" />
      </div>
    </div>
  );
}
