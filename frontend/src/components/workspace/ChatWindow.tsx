import React, { useState, useRef, useEffect, type FormEvent, type ChangeEvent, type KeyboardEvent } from 'react';
import { ScrollArea } from "../../components/ui/scroll-area";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import { 
  Terminal, 
  Check, 
  AlertCircle,
  Zap,
  Play,
  ChevronUp,
  ChevronDown,
  Rocket,
  Plus,
  Mic,
  ArrowRight,
  Square
} from "lucide-react";
import { ThoughtBlock } from "./ThoughtBlock";
import { ActionCard } from "./ActionCard";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { vscDarkPlus, vs } from "react-syntax-highlighter/dist/esm/styles/prism";
import { runSimulation, type BenchmarkObjectives } from "../../api/client";
import { ObjectivesForm } from "./ObjectivesForm";
import ConnectionError from "../shared/ConnectionError";
import { Button } from "../ui/button";
import { ContextCards } from "./ContextCards";
import { FeedbackSystem } from "./FeedbackSystem";
import { useEpisodes } from "../../context/EpisodeContext";
import { useTheme } from "../../context/ThemeContext";
import { cn } from "../../lib/utils";
import type { TopologyNode } from "../visualization/ModelBrowser";
import { getFileIconInfo } from "../../lib/fileIcons";

// Internal helper for rendering highlighted content
const HighlightedContent = ({ content, language = 'text' }: { content: string, language?: string }) => {
        const { theme } = useTheme();
        const { addToContext, setActiveArtifactId, selectedEpisode } = useEpisodes();
        
        if (!content) return null;
        
        // Custom renderer for mentions in markdown/text
        const renderWithMentions = (text: string) => {
            const regex = /(@([a-zA-Z0-9_\-\.]+)(?::L?(\d+)(?:-L?(\d+))?)?)/g;
            const parts = [];
            let lastIndex = 0;
            let match;
            
            while ((match = regex.exec(text)) !== null) {
                const [full, , name, start, end] = match;
                parts.push(text.substring(lastIndex, match.index));
                
                parts.push(
                    <button
                        key={match.index}
                        className="text-primary hover:underline font-mono font-bold px-1 rounded bg-primary/10"
                        onClick={() => {
                            const asset = selectedEpisode?.assets?.find(a => a.s3_path.endsWith(name) || a.s3_path === name);
                            if (asset) {
                                setActiveArtifactId(asset.id.toString());
                                if (start) {
                                    addToContext({
                                        id: `steer-${asset.id}-${start}-${end || start}`,
                                        type: 'code',
                                        label: full.substring(1),
                                        metadata: { 
                                            path: asset.s3_path, 
                                            start: parseInt(start), 
                                            end: end ? parseInt(end) : parseInt(start) 
                                        }
                                    });
                                }
                            } else {
                                // Assume part mention
                                addToContext({
                                    id: `cad-${name}`,
                                    type: 'cad',
                                    label: name,
                                    metadata: { part: name }
                                });
                            }
                        }}
                    >
                        {full}
                    </button>
                );
                lastIndex = regex.lastIndex;
            }
            parts.push(text.substring(lastIndex));
            return parts;
        };

        // Basic detection for JSON strings
        const lang = (content.trim().startsWith('{') || content.trim().startsWith('[')) ? 'json' : language;

        if (lang === 'markdown' || lang === 'text') {
            return (
                <div className="text-[13px] leading-relaxed text-foreground/90 whitespace-pre-wrap py-1">
                    {renderWithMentions(content)}
                </div>
            );
        }

        return (
                <SyntaxHighlighter
                        language={lang}
                        style={theme === 'dark' ? vscDarkPlus : vs}
                        customStyle={{
                                margin: 0,
                                padding: '0.5rem',
                                background: theme === 'dark' ? 'rgba(0,0,0,0.2)' : 'rgba(0,0,0,0.05)',
                                fontSize: '10px',
                                borderRadius: '4px',
                                border: theme === 'dark' ? '1px solid rgba(255,255,255,0.05)' : '1px solid rgba(0,0,0,0.05)'
                        }}
                        wrapLines={true}
                        wrapLongLines={true}
                >
                        {content}
                </SyntaxHighlighter>
        );
};

interface Suggestion {
  id: string;
  name: string;
  type: 'file' | 'part';
  icon: React.ComponentType<{ className?: string }>;
  original: any;
}

interface ChatWindowProps {
  traces?: TraceResponse[];
  task?: string;
  isRunning?: boolean;
  isConnected?: boolean;
  topologyNodes?: TopologyNode[];
}

export default function ChatWindow({
  traces,
  task,
  isRunning,
  isConnected = true,
  topologyNodes = []
}: ChatWindowProps) {
  const scrollRef = useRef<HTMLDivElement>(null);
  const { 
      isCreationMode, 
      startAgent, 
      continueAgent,
      steerAgent,
      confirmBenchmark,
      interruptAgent, 
      selectedEpisode, 
      updateObjectives, 
      selectedContext,
      addToContext,
      clearContext
  } = useEpisodes();
  const [prompt, setPrompt] = useState("");
  const [selectedBenchmarkId, setSelectedBenchmarkId] = useState<string>("");

  const [objectives, setObjectives] = useState<BenchmarkObjectives>({});
  const [showObjectives, setShowObjectives] = useState(false);
  const [showFeedbackModal, setShowFeedbackModal] = useState(false);
  const [showSuggestions, setShowSuggestions] = useState(false);
  const [suggestions, setSuggestions] = useState<Suggestion[]>([]);
  const [selectedIndex, setSelectedIndex] = useState(0);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  const location = window.location;
  const isBenchmarkPath = location.pathname === '/benchmark';


  // Reset objectives when episode changes
  useEffect(() => {
      if (selectedEpisode?.metadata_vars?.custom_objectives) {
          setObjectives(selectedEpisode.metadata_vars.custom_objectives as BenchmarkObjectives);
      } else if (isCreationMode) {
          setObjectives({});
      }
  }, [selectedEpisode, isCreationMode]);

  const handleSubmit = async (e?: FormEvent | KeyboardEvent) => {
    if (e) e.preventDefault();
    if (prompt.trim()) {
      // 1. Extract mentions and code references from selectedContext
      const mentions: string[] = [];
      const code_references: any[] = [];
      const selections: any[] = [];

      selectedContext.forEach(item => {
        if (item.type === 'cad') {
          mentions.push(item.label);
        } else if (item.type === 'code') {
          code_references.push({
            file_path: item.metadata?.path,
            start_line: Number(item.metadata?.start || item.metadata?.line),
            end_line: Number(item.metadata?.end || item.metadata?.line)
          });
        }
      });

      // 2. Parse line-range steering from text: @filename:L1-L2
      const steeringRegex = /@([^:\n\s]+):L?(\d+)(?:-L?(\d+))?/g;
      const matches = [...prompt.matchAll(steeringRegex)];
      
      const sessionAssets = selectedEpisode?.assets || [];
      matches.forEach(match => {
        const [, filename, start, end] = match;
        const asset = sessionAssets.find(a => a.s3_path.endsWith(filename) || a.s3_path === filename);
        if (asset) {
            const ref = {
                file_path: asset.s3_path,
                start_line: parseInt(start),
                end_line: end ? parseInt(end) : parseInt(start)
            };
            // Avoid duplicates
            if (!code_references.some(r => r.file_path === ref.file_path && r.start_line === ref.start_line && r.end_line === ref.end_line)) {
                code_references.push(ref);
            }
        }
      });

      // 3. Extract simple mentions from text (e.g. @part_name)
      const mentionRegex = /@([a-zA-Z0-9_\-\.]+)(?![:\d])/g;
      const mentionMatches = [...prompt.matchAll(mentionRegex)];
      mentionMatches.forEach(match => {
        const m = match[1];
        if (!mentions.includes(m)) {
          mentions.push(m);
        }
      });

      const metadata = {
        ...(selectedBenchmarkId ? { benchmark_id: selectedBenchmarkId } : {}),
        mentions,
        code_references,
        selections,
        context_items: selectedContext
      };
      
      if (selectedEpisode && !isCreationMode) {
        if (isRunning) {
          await steerAgent(selectedEpisode.id, prompt, metadata);
        } else {
          await continueAgent(selectedEpisode.id, prompt, metadata);
        }
      } else {
        await startAgent(prompt, objectives, Object.keys(metadata).length > 0 ? metadata : undefined);
      }
      
      setPrompt("");
      setSelectedBenchmarkId("");
      clearContext();
      setShowSuggestions(false);
    }
  };

  const handleMention = (value: string, selectionStart: number) => {
    // 1. Line range detection for immediate feedback (Story 4)
    const lineRegex = /@([^:\n\s]+):L?(\d+)(?:-L?(\d+))?/g;
    const matches = [...value.matchAll(lineRegex)];
    
    const sessionAssets = selectedEpisode?.assets || [];
    matches.forEach(match => {
        const [, filename, start, end] = match;
        const asset = sessionAssets.find(a => a.s3_path.endsWith(filename) || a.s3_path === filename);
        if (asset) {
            const contextId = `steer-${asset.id}-${start}-${end || start}`;
            if (!selectedContext.find(i => i.id === contextId)) {
                addToContext({
                    id: contextId,
                    type: 'code',
                    label: `${filename}:${start}${end ? '-' + end : ''}`,
                    metadata: { 
                        path: asset.s3_path, 
                        start: parseInt(start), 
                        end: end ? parseInt(end) : parseInt(start) 
                    }
                });
            }
        }
    });

    // 2. Existing @-mention logic
    const lastAt = value.lastIndexOf('@', selectionStart - 1);
    const filter = value.substring(lastAt + 1, selectionStart);
    
    if (lastAt !== -1 && !/\s/.test(filter)) {
        setShowSuggestions(true);
        
        // Populate suggestions from BOM and Assets
        const items: Suggestion[] = [];
        
        // Add Assets (Files)
        (selectedEpisode?.assets || []).forEach(asset => {
            const name = asset.s3_path.split('/').pop() || asset.s3_path;
            if (name.toLowerCase().includes(filter.toLowerCase())) {
                const iconInfo = getFileIconInfo(name, asset.asset_type);
                items.push({ id: `file-${asset.id}`, name, type: 'file', icon: iconInfo.icon, original: asset });
            }
        });

        // Add BOM Nodes (Parts)
        const collectParts = (nodes: TopologyNode[]) => {
            nodes.forEach(node => {
                if (node.name.toLowerCase().includes(filter.toLowerCase())) {
                    items.push({ id: `part-${node.id}`, name: node.name, type: 'part', icon: Zap, original: node });
                }
                if (node.children) collectParts(node.children);
            });
        };
        collectParts(topologyNodes);

        setSuggestions(items.slice(0, 10)); // Limit to 10
        setSelectedIndex(0);
    } else {
        setShowSuggestions(false);
    }
  };

  const insertSuggestion = (suggestion: Suggestion) => {
    const selectionStart = inputRef.current?.selectionStart || 0;
    const lastAt = prompt.lastIndexOf('@', selectionStart - 1);
    
    const newPrompt = prompt.substring(0, lastAt + 1) + suggestion.name + prompt.substring(selectionStart);
    setPrompt(newPrompt);
    setShowSuggestions(false);
    
    // Auto-add to context for immediate feedback (Story 4)
    if (suggestion.type === 'part') {
        addToContext({
            id: `cad-${suggestion.original.name}`,
            type: 'cad',
            label: suggestion.original.name,
            metadata: { part: suggestion.original.name }
        });
    } else if (suggestion.type === 'file') {
        addToContext({
            id: `code-${suggestion.original.s3_path}`,
            type: 'code',
            label: suggestion.original.s3_path.split('/').pop() || suggestion.original.s3_path,
            metadata: { path: suggestion.original.s3_path }
        });
    }

    setTimeout(() => {
        if (inputRef.current) {
            const newPos = lastAt + 1 + suggestion.name.length;
            inputRef.current.selectionStart = newPos;
            inputRef.current.selectionEnd = newPos;
            inputRef.current.focus();
        }
    }, 0);
  };

  // Auto-scroll to bottom when new traces arrive
  useEffect(() => {
    if (traces && traces.length > 0) {
      const timeout = setTimeout(() => {
        const scrollContainer = scrollRef.current?.querySelector('[data-radix-scroll-area-viewport]');
        if (scrollContainer) {
          const { scrollTop, scrollHeight, clientHeight } = scrollContainer;
          const isAtBottom = scrollHeight - scrollTop - clientHeight < 150;
          if (isAtBottom) {
            scrollContainer.scrollTop = scrollContainer.scrollHeight;
          }
        }
      }, 100);
      return () => clearTimeout(timeout);
    }
  }, [traces]);

  const isPlanned = selectedEpisode?.metadata_vars?.detailed_status === 'planned';
  const showExecutionPlan = (selectedEpisode?.plan || isPlanned) && !isRunning && selectedEpisode?.status !== 'completed';

  return (
    <div className="flex flex-col h-full bg-background relative overflow-hidden">
      {!isConnected && <ConnectionError className="absolute inset-0 z-50" />}
      
      {/* Header */}
      <div className="flex items-center justify-between p-3 border-b border-white/5 bg-background shrink-0">
          <div className="flex items-center gap-2">
            <Zap className="h-4 w-4 text-primary" />
            <span className="text-[11px] font-bold tracking-tight opacity-70">Engineer Workspace</span>
          </div>
      </div>

      {/* Main Content Area */}
      <div className="flex-1 overflow-hidden flex flex-col relative">
        <ScrollArea className="flex-1" ref={scrollRef}>
            <div className="p-4 flex flex-col min-h-full">
                {/* Creation Mode Prompt */}
                {isCreationMode && !task && (
                    <div className="mb-6 p-4 bg-primary/5 rounded-xl border-2 border-dashed border-primary/20 animate-in fade-in zoom-in-95 duration-500">
                        <div className="flex flex-col items-center text-center gap-3">
                            <div className="size-10 flex items-center justify-center bg-primary/20 rounded-full text-primary">
                                {isBenchmarkPath ? <Rocket className="h-5 w-5" /> : <Zap className="h-5 w-5" />}
                            </div>
                            <div className="space-y-1">
                                <h3 className="text-xs font-black uppercase tracking-widest text-primary">
                                    {isBenchmarkPath ? 'Benchmark Creation Mode' : 'New Design Session'}
                                </h3>
                                <p className="text-[10px] text-muted-foreground leading-relaxed max-w-[200px]">
                                    {isBenchmarkPath 
                                        ? 'Enter a high-level prompt below to begin generating a new benchmark plan.'
                                        : 'Describe the part or mechanism you want to design to start a new session.'
                                    }
                                </p>
                            </div>
                        </div>
                    </div>
                )}

                {/* Initial Task Message (User Style) */}
                {task && (
                    <div className="flex justify-end mb-8 animate-in slide-in-from-right-4 fade-in duration-300">
                        <div className="max-w-[85%] bg-muted/40 rounded-2xl p-4 shadow-sm border border-border/10 relative group">
                            <div className="text-[13px] leading-relaxed text-foreground whitespace-pre-wrap">
                                {task}
                            </div>
                            <button className="absolute bottom-2 right-4 opacity-0 group-hover:opacity-40 transition-opacity">
                                <Rocket className="h-3.5 w-3.5" />
                            </button>
                        </div>
                    </div>
                )}

                {/* Messages / Traces */}
                <div className="space-y-4 max-w-3xl mx-auto w-full">
                  {traces && traces.length > 0 ? (
                      traces.map(trace => {
                          const type = trace.trace_type as string;
                          if (type === 'llm_thought' || type === 'thought') {
                            return <ThoughtBlock key={trace.id} duration={Math.floor(Math.random() * 5) + 1} content={trace.content || ""} />;
                          }
                          if (type === 'tool_start') {
                              return <ActionCard key={trace.id} trace={trace} />;
                          }
                          if (type === 'llm_end' && trace.content) {
                            return (
                                <div key={trace.id} className="relative group/msg">
                                    <HighlightedContent content={trace.content} language="markdown" />
                                    <div className="flex items-center gap-2 mt-1 opacity-0 group-hover/msg:opacity-100 transition-opacity">
                                        <button 
                                            onClick={() => setShowFeedbackModal(true)}
                                            className="text-[10px] text-muted-foreground hover:text-foreground flex items-center gap-1"
                                        >
                                            <AlertCircle className="h-3 w-3" />
                                            Feedback
                                        </button>
                                    </div>
                                </div>
                            );
                          }
                          return null;
                      })
                  ) : (
                      <div className="flex flex-col items-center justify-center py-20 gap-2 opacity-10 h-full">
                         <Terminal className="h-8 w-8" />
                         <span className="text-[10px] uppercase font-bold tracking-widest">Awaiting interaction...</span>
                      </div>
                  )}
                </div>

                {/* Execution Plan Card (Inline) */}
                {showExecutionPlan && (
                    <div className="mt-8 mb-4 p-4 rounded-xl border-2 border-primary/30 bg-primary/5 shadow-2xl animate-in fade-in slide-in-from-bottom-4 duration-500">
                        <div className="flex flex-col items-center text-center gap-3">
                            <div className="size-12 flex items-center justify-center bg-primary/20 rounded-full text-primary ring-4 ring-primary/10">
                                <Check className="h-6 w-6 stroke-[3]" />
                            </div>
                            <div className="space-y-1">
                                <h3 className="text-sm font-black uppercase tracking-tighter text-foreground">Execution Plan Ready</h3>
                                <p className="text-xs text-muted-foreground leading-relaxed max-w-[240px]">
                                    I have drafted the implementation strategy. Please review it in the explorer and confirm to proceed.
                                </p>
                            </div>
                            <div className="flex gap-2 w-full mt-2">
                                <Button 
                                    className="flex-1 bg-primary hover:bg-primary/90 text-primary-foreground font-black text-[10px] uppercase tracking-widest h-10 shadow-lg shadow-primary/20"
                                    onClick={async () => {
                                        if (!selectedEpisode) return;
                                        try {
                                            if (isPlanned) {
                                                await confirmBenchmark(selectedEpisode.id);
                                            } else {
                                                const sessionId = `sim-${Math.random().toString(36).substring(2, 10)}`;
                                                await runSimulation(sessionId);
                                            }
                                        } catch (e) {
                                            console.error("Failed to start implementation", e);
                                        }
                                    }}
                                >
                                    <Play className="h-3 w-3 mr-2 fill-current" />
                                    Confirm & Start
                                </Button>
                                <Button 
                                    variant="outline"
                                    className="px-3 border-border/50 hover:bg-muted/50 text-[10px] uppercase font-bold text-muted-foreground"
                                    onClick={() => {
                                      const input = document.getElementById('chat-input');
                                      input?.focus();
                                    }}
                                >
                                    Request Changes
                                </Button>
                            </div>
                        </div>
                    </div>
                )}
                
                {/* Visual indicator for active streaming */}
                {isRunning && (
                    <div className="mt-4 flex items-center gap-2 text-primary animate-pulse pl-1 opacity-50 text-[10px] font-mono">
                        <div className="size-1 bg-primary rounded-full" />
                        <span>Agent is thinking...</span>
                    </div>
                )}

                {/* Failure Message */}
                {selectedEpisode?.status === 'failed' && (
                    <div className="mt-6 p-3 bg-red-500/10 rounded-lg border border-red-500/20 shadow-sm">
                        <div className="flex items-center gap-2 mb-2 text-red-500">
                            <AlertCircle className="h-4 w-4" />
                            <span className="text-[9px] font-black uppercase tracking-widest">Terminal failure</span>
                        </div>
                        <div className="text-[11px] text-red-400 font-mono whitespace-pre-wrap">
                            {selectedEpisode.validation_logs && selectedEpisode.validation_logs.length > 0 
                                ? selectedEpisode.validation_logs.join('\n')
                                : "The agent encountered an unrecoverable error during execution."}
                        </div>
                    </div>
                )}
                {/* Session Feedback Modal */}
                {showFeedbackModal && selectedEpisode && (
                    <FeedbackSystem 
                        episodeId={selectedEpisode.id} 
                        onClose={() => setShowFeedbackModal(false)}
                    />
                )}
            </div>
        </ScrollArea>

        {/* Steering / Input Area */}
        <div className="p-4 max-w-3xl mx-auto w-full relative">
            <ContextCards />
            
            <div className="bg-muted/30 border border-white/5 rounded-2xl p-3 shadow-sm focus-within:ring-1 focus-within:ring-primary/20 transition-all">
                <form onSubmit={handleSubmit} className="flex flex-col gap-2">
                    <textarea 
                       id="chat-input"
                       placeholder={isRunning ? "Steer the agent..." : "Ask anything (Ctrl+L), @ to mention, / for workflows"}
                       className="w-full bg-transparent border-none focus:ring-0 text-[14px] leading-relaxed resize-none min-h-[40px] max-h-48"
                       rows={1}
                       ref={inputRef}
                       value={prompt}
                       onChange={(e: ChangeEvent<HTMLTextAreaElement>) => {
                           const val = e.target.value;
                           setPrompt(val);
                           handleMention(val, e.target.selectionStart || 0);
                           e.target.style.height = 'auto';
                           e.target.style.height = e.target.scrollHeight + 'px';
                       }}
                       onKeyDown={(e: KeyboardEvent<HTMLTextAreaElement>) => {
                           if (showSuggestions && suggestions.length > 0) {
                               if (e.key === 'ArrowDown') {
                                   e.preventDefault();
                                   setSelectedIndex((prev: number) => (prev + 1) % suggestions.length);
                                   return;
                               }
                               if (e.key === 'ArrowUp') {
                                   e.preventDefault();
                                   setSelectedIndex((prev: number) => (prev - 1 + suggestions.length) % suggestions.length);
                                   return;
                               }
                               if (e.key === 'Tab' || e.key === 'Enter') {
                                   e.preventDefault();
                                   insertSuggestion(suggestions[selectedIndex]);
                                   return;
                               }
                               if (e.key === 'Escape') {
                                   setShowSuggestions(false);
                                   return;
                               }
                           }
                           if (e.key === 'Enter' && !e.shiftKey) {
                                e.preventDefault();
                                handleSubmit(e);
                           }
                       }}
                    />

                    {/* Autocomplete Suggestions */}
                    {showSuggestions && suggestions.length > 0 && (
                        <div className="absolute bottom-[calc(100%+8px)] left-4 right-4 bg-card border border-border shadow-2xl rounded-xl overflow-hidden z-[100] animate-in slide-in-from-bottom-2 duration-200">
                            <div className="p-2 border-b border-border bg-muted/20">
                                <span className="text-[9px] font-black uppercase tracking-widest text-muted-foreground opacity-70">Suggestions</span>
                            </div>
                            <div className="max-h-48 overflow-y-auto p-1">
                                {suggestions.map((s, idx) => (
                                    <button
                                        key={s.id}
                                        type="button"
                                        onClick={(e) => {
                                            e.preventDefault();
                                            insertSuggestion(s);
                                        }}
                                        className={cn(
                                            "flex items-center gap-2 w-full text-left px-3 py-2 rounded-lg text-[12px] transition-all",
                                            idx === selectedIndex ? "bg-primary/20 text-primary" : "hover:bg-muted/50 text-muted-foreground hover:text-foreground"
                                        )}
                                    >
                                        <s.icon className="h-3.5 w-3.5 shrink-0 opacity-70" />
                                        <span className="truncate flex-1">{s.name}</span>
                                        <span className="text-[9px] font-mono opacity-30 uppercase">{s.type}</span>
                                    </button>
                                ))}
                            </div>
                        </div>
                    )}
                    
                    <div className="flex items-center justify-between mt-1">
                        <div className="flex items-center gap-1.5">
                            <Button variant="ghost" size="icon" className="h-7 w-7 rounded-lg hover:bg-white/5">
                                <Plus className="h-4 w-4 opacity-40" />
                            </Button>
                            
                            <Button 
                                variant="ghost" 
                                size="sm" 
                                className="h-7 px-2 gap-1 rounded-lg hover:bg-white/5 text-[11px] font-medium text-muted-foreground"
                                onClick={(e) => {
                                    e.preventDefault();
                                    setShowObjectives(!showObjectives);
                                }}
                            >
                                <Zap className="h-3 w-3 opacity-40" />
                                Planning
                                {showObjectives ? <ChevronDown className="h-3 w-3 ml-0.5 opacity-40" /> : <ChevronUp className="h-3 w-3 ml-0.5 opacity-40" />}
                            </Button>

                            <Button 
                                variant="ghost" 
                                size="sm" 
                                className="h-7 px-2 gap-1 rounded-lg hover:bg-white/5 text-[11px] font-medium text-muted-foreground"
                            >
                                <Rocket className="h-3 w-3 opacity-40" />
                                Claude Opus 4.6
                            </Button>
                        </div>

                        <div className="flex items-center gap-1.5">
                            <Button variant="ghost" size="icon" className="h-8 w-8 rounded-lg hover:bg-white/5">
                                <Mic className="h-4 w-4 opacity-40" />
                            </Button>
                            
                            <Button 
                               type={isRunning ? "button" : "submit"}
                               size="icon" 
                               disabled={!isRunning && !prompt.trim()}
                               className={cn(
                                   "h-8 w-8 rounded-full transition-all flex items-center justify-center",
                                   isRunning 
                                    ? "bg-red-500/20 text-red-500 hover:bg-red-500/30" 
                                    : "bg-foreground text-background hover:opacity-90 disabled:bg-muted/20 disabled:text-muted-foreground"
                               )}
                               aria-label={isRunning ? "Stop Agent" : "Send Message"}
                               onClick={(e) => {
                                   if (isRunning && selectedEpisode) {
                                       e.preventDefault();
                                       interruptAgent(selectedEpisode.id);
                                   }
                               }}
                            >
                                {isRunning ? <Square className="h-3.5 w-3.5 fill-current" aria-label="Stop Icon" /> : <ArrowRight className="h-4 w-4" aria-label="Send Icon" />}
                            </Button>
                        </div>
                    </div>
                </form>
            </div>

            {showObjectives && (
                <div className="absolute bottom-full left-0 right-0 mb-2 px-4 animate-in slide-in-from-bottom-2 duration-200">
                    <div className="bg-background border border-border shadow-2xl rounded-xl overflow-hidden">
                        <ObjectivesForm 
                            objectives={objectives} 
                            onChange={setObjectives} 
                            onUpdate={() => updateObjectives(objectives)}
                            showUpdate={!isCreationMode && !!selectedEpisode}
                            disabled={isRunning}
                        />
                    </div>
                </div>
            )}
        </div>
      </div>
    </div>
  );
}
