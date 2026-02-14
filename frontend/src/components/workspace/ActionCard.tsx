import { Search, List, Eye, FileEdit, PlayCircle, Zap } from "lucide-react";
import { cn } from "../../lib/utils";
import { getFileIconInfo } from "../../lib/fileIcons";
import { useEpisodes } from "../../context/EpisodeContext";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";

interface ActionCardProps {
  trace: TraceResponse;
  resultCount?: number;
  className?: string;
}

export function ActionCard({ trace, resultCount, className }: ActionCardProps) {
  const { setActiveArtifactId, selectedEpisode } = useEpisodes();
  const toolName = (trace.name || "").toLowerCase();
  
  if (trace.trace_type !== 'tool_start') return null;

  let args: any = {};
  try {
    if (trace.content) args = JSON.parse(trace.content);
  } catch (e) {
    // Basic extraction for non-JSON content - handles quoted and unquoted paths
    const match = trace.content?.match(/(?:TargetFile|AbsolutePath|path)["']?\s*[:=]\s*["']?([^"'\s,]+)["']?/);
    if (match) args.path = match[1];
  }

  const filePath = args.TargetFile || args.AbsolutePath || args.path || "";
  const fileName = filePath.split('/').pop() || "";
  
  const getToolIcon = () => {
    if (toolName.includes('search')) return Search;
    if (toolName.includes('list') || toolName.includes('ls')) return List;
    if (toolName.includes('read') || toolName.includes('view') || toolName.includes('analyze')) {
        return Eye;
    }
    if (toolName.includes('write') || toolName.includes('replace')) return FileEdit;
    if (toolName.includes('run') || toolName.includes('exec')) return PlayCircle;
    return Zap;
  };

  const { icon: FileIcon, color: iconColor } = fileName 
    ? getFileIconInfo(fileName) 
    : { icon: getToolIcon(), color: undefined };

  const handleActionClick = () => {
    if (!filePath || !selectedEpisode) return;
    const asset = selectedEpisode.assets?.find(a => 
      a.s3_path.toLowerCase().endsWith(filePath.toLowerCase()) || 
      a.s3_path.toLowerCase() === filePath.toLowerCase()
    );
    if (asset) setActiveArtifactId(asset.id.toString());
    else if (filePath.toLowerCase().endsWith('plan.md')) setActiveArtifactId('plan');
  };

  const getLabel = () => {
    if (toolName.includes('analyze')) return "Analyzed";
    if (toolName.includes('search')) return "Searched";
    if (toolName.includes('read') || toolName.includes('view')) return "Read";
    if (toolName.includes('write') || toolName.includes('replace')) return "Edited";
    if (toolName.includes('list')) return "Viewed";
    return "Used";
  };

  return (
    <div 
      onClick={handleActionClick}
      className={cn(
        "flex items-center gap-2 py-0.5 transition-colors cursor-pointer group/action text-[11px] leading-tight",
        "text-muted-foreground/60 hover:text-foreground",
        className
      )}
    >
      <span className="font-medium whitespace-nowrap">{getLabel()}</span>
      <FileIcon 
        className="h-3.5 w-3.5 shrink-0 transition-opacity" 
        style={{ color: iconColor, opacity: iconColor ? 0.8 : 0.4 }}
      />
      {fileName && (
          <span className="font-mono text-primary/70 group-hover/action:text-primary transition-colors truncate">
            {fileName}
            {args.line && <span className="text-muted-foreground/40 ml-0.5">#L{args.line}</span>}
          </span>
      )}
      {!fileName && trace.name && <span className="font-mono text-primary/70 truncate">{trace.name}</span>}
      
      {resultCount !== undefined && (
        <span className="ml-auto text-[10px] text-muted-foreground/30 font-mono">
          {resultCount} results
        </span>
      )}
    </div>
  );
}
