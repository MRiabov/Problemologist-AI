import { memo } from "react";
import { Search, List, Eye, FileEdit, PlayCircle, Zap, AlertCircle } from "lucide-react";
import { cn } from "../../lib/utils";
import { getFileIconInfo } from "../../lib/fileIcons";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import { TraceType } from "../../api/generated/models/TraceType";

interface ActionCardProps {
  trace: TraceResponse;
  resultCount?: number;
  className?: string;
  assets?: AssetResponse[];
  setActiveArtifactId: (id: string | null) => void;
}

export const ActionCard = memo(({ trace, resultCount, className, assets, setActiveArtifactId }: ActionCardProps) => {
  const toolName = (trace.name || "").toLowerCase();
  const traceMetadata = (trace.metadata_vars ?? {}) as Record<string, any>;
  const output = traceMetadata.output as string | undefined;
  const hasError = !!(traceMetadata.error || (output && output.includes("exit_code") && !output.includes("exit_code': 0") && !output.includes('exit_code": 0') && !output.includes("exit_code=0")));
  
  if (trace.trace_type !== TraceType.TOOL_START) return null;

  let args: Record<string, any> = {};
  try {
    if (trace.content) args = JSON.parse(trace.content);
  } catch {
    // Basic extraction for non-JSON content - handles quoted and unquoted paths
    const match = trace.content?.match(/(?:TargetFile|AbsolutePath|path)["']?\s*[:=]\s*["']?([^"'\s,]+)["']?/);
    if (match) args.path = match[1];
  }

  const findPath = () => {
    const fromRoot = args.TargetFile || args.AbsolutePath || args.path || args.file_path || args.filePath;
    if (typeof fromRoot === 'string' && fromRoot.length > 0) return fromRoot;

    const nestedKwargs = args.kwargs as Record<string, any> | undefined;
    const fromKwargs = nestedKwargs?.TargetFile || nestedKwargs?.AbsolutePath || nestedKwargs?.path || nestedKwargs?.file_path || nestedKwargs?.filePath;
    if (typeof fromKwargs === 'string' && fromKwargs.length > 0) return fromKwargs;

    const positionalArgs = Array.isArray(args.args) ? args.args : [];
    if (positionalArgs.length > 0 && typeof positionalArgs[0] === 'string') {
      return positionalArgs[0];
    }

    return "";
  };

  const filePath = findPath();
  const isDirectoryTool = toolName.includes('list') || toolName.includes('ls');
  const getDisplayName = () => {
    if (!filePath) return "";
    const normalized = filePath === "/" ? "/" : filePath.replace(/\/+$/, "");
    if (normalized === "/") return "/";
    const parts = normalized.split('/').filter(Boolean);
    const base = parts[parts.length - 1] || normalized;
    return isDirectoryTool ? base : (base || normalized);
  };
  const fileName = getDisplayName();
  
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
    if (!filePath) return;
    const asset = assets?.find(a =>
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
      data-testid="tool-activity-row"
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
      
      {hasError && (
        <span className="ml-auto text-[10px] text-red-400 font-mono flex items-center gap-1">
          <AlertCircle className="h-3 w-3" />
          Failed
        </span>
      )}

      {resultCount !== undefined && !hasError && (
        <span className="ml-auto text-[10px] text-muted-foreground/30 font-mono">
          {resultCount} results
        </span>
      )}
    </div>
  );
});

ActionCard.displayName = 'ActionCard';
