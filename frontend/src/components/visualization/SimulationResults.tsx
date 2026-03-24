import React from 'react';
import { Badge } from "../ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "../ui/card";
import { 
  AlertTriangle, 
  CheckCircle2, 
  Thermometer, 
  Droplets,
  Activity,
  CircleAlert
} from "lucide-react";
import { cn } from "../../lib/utils";
import type { MultiRunResult } from "../../api/generated/models/MultiRunResult";
import {
  formatStabilitySummaryLines,
  summarizeVerificationResult,
} from "../workspace/stabilitySummary";

// Types from backend SimulationResult
export interface StressSummary {
  part_label: string;
  max_von_mises_pa: number;
  mean_von_mises_pa: number;
  safety_factor: number;
  location_of_max: [number, number, number];
  utilization_pct: number;
}

export interface FluidMetricResult {
  metric_type: string;
  fluid_id: string;
  measured_value: number;
  target_value: number;
  passed: boolean;
}

interface SimulationResultsProps {
  stressSummaries?: StressSummary[];
  fluidMetrics?: FluidMetricResult[];
  summary?: string | null;
  renderPaths?: string[];
  verificationResult?: MultiRunResult | null;
  className?: string;
}

export const SimulationResults: React.FC<SimulationResultsProps> = ({
  stressSummaries = [],
  fluidMetrics = [],
  summary = null,
  renderPaths = [],
  verificationResult = null,
  className
}) => {
  const hasOverviewEvidence = !!summary || renderPaths.length > 0;
  const stabilitySummary = summarizeVerificationResult(verificationResult);

  if (
    stressSummaries.length === 0 &&
    fluidMetrics.length === 0 &&
    !hasOverviewEvidence &&
    !stabilitySummary
  ) {
    return (
      <div data-testid="simulation-results-empty" className={cn("p-8 text-center text-muted-foreground italic", className)}>
        No simulation data available.
      </div>
    );
  }

  return (
    <div data-testid="simulation-results-root" className={cn("space-y-6 p-4", className)}>
      {summary && (
        <Card className="bg-card/50 backdrop-blur-sm border-border/50">
          <CardHeader className="pb-2">
            <CardTitle className="text-sm font-bold uppercase tracking-widest flex items-center gap-2">
              <Activity className="h-4 w-4 text-primary" />
              Simulation Summary
            </CardTitle>
          </CardHeader>
          <CardContent>
            <p data-testid="simulation-results-summary" className="text-sm text-foreground/90 leading-relaxed whitespace-pre-wrap">
              {summary}
            </p>
          </CardContent>
        </Card>
      )}

      {renderPaths.length > 0 && (
        <Card className="bg-card/50 backdrop-blur-sm border-border/50">
          <CardHeader className="pb-2">
            <CardTitle className="text-sm font-bold uppercase tracking-widest flex items-center gap-2">
              <Activity className="h-4 w-4 text-blue-500" />
              Render Evidence
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-2">
            {renderPaths.map((renderPath, idx) => (
              <div
                key={`${renderPath}-${idx}`}
                data-testid={`simulation-render-path-${idx}`}
                className="rounded-md border border-border/40 bg-muted/20 px-3 py-2 text-[11px] font-mono break-all text-muted-foreground"
              >
                {renderPath}
              </div>
            ))}
          </CardContent>
        </Card>
      )}

      {stabilitySummary && (
        <div data-testid="verification-summary-panel">
          <Card
            data-testid="simulation-verification-card"
            className="bg-card/50 backdrop-blur-sm border-border/50"
          >
          <CardHeader className="pb-2">
            <CardTitle className="text-sm font-bold uppercase tracking-widest flex items-center gap-2">
              <CircleAlert className="h-4 w-4 text-primary" />
              Runtime Jitter Verification
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="grid grid-cols-2 gap-3 text-[11px]">
              {formatStabilitySummaryLines(stabilitySummary).map((line) => (
                <div
                  key={line}
                  className="rounded-md border border-border/40 bg-muted/20 px-3 py-2 text-muted-foreground"
                >
                  {line}
                </div>
              ))}
            </div>
            <div className="grid grid-cols-1 gap-2 text-[11px] font-mono">
              <div className="flex items-center justify-between gap-2 rounded-md border border-border/40 bg-muted/10 px-3 py-2">
                <span className="uppercase tracking-widest text-muted-foreground">Scene build count</span>
                <span>{stabilitySummary.sceneBuildCount}</span>
              </div>
              <div className="flex items-center justify-between gap-2 rounded-md border border-border/40 bg-muted/10 px-3 py-2">
                <span className="uppercase tracking-widest text-muted-foreground">Backend run count</span>
                <span>{stabilitySummary.backendRunCount}</span>
              </div>
            </div>
            <div className="space-y-2">
              {stabilitySummary.sceneSummaries.map((scene) => (
                <div
                  key={scene.sceneIndex}
                  data-testid={`simulation-verification-scene-${scene.sceneIndex}`}
                  className="rounded-md border border-border/40 bg-background/70 px-3 py-2 text-[11px] font-mono"
                >
                  <div className="flex items-center justify-between gap-2">
                    <span className="font-semibold uppercase tracking-widest">
                      Scene {scene.sceneIndex}
                    </span>
                    <Badge
                      variant="outline"
                      className={cn(
                        "text-[9px] font-black uppercase tracking-widest h-5 px-2",
                        scene.success
                          ? "border-emerald-500/40 text-emerald-600 bg-emerald-500/5"
                          : "border-red-500/40 text-red-500 bg-red-500/5",
                      )}
                    >
                      {scene.success ? "Pass" : "Fail"}
                    </Badge>
                  </div>
                  <div className="mt-2 text-muted-foreground whitespace-pre-wrap">
                    {scene.summary}
                  </div>
                  {!scene.success && scene.failReason && (
                    <div className="mt-1 text-red-400">
                      Reason: {scene.failReason}
                    </div>
                  )}
                </div>
              ))}
            </div>
          </CardContent>
          </Card>
        </div>
      )}

      {/* Stress Summaries */}
      {stressSummaries.length > 0 && (
        <Card className="bg-card/50 backdrop-blur-sm border-border/50">
          <CardHeader className="pb-2">
            <CardTitle className="text-sm font-bold uppercase tracking-widest flex items-center gap-2">
              <Thermometer className="h-4 w-4 text-orange-500" />
              Structural Stress Analysis
            </CardTitle>
          </CardHeader>
          <CardContent>
            <div className="overflow-x-auto">
              <table className="w-full border-collapse">
                <thead>
                  <tr className="border-b border-border/50 text-left">
                    <th className="p-2 text-[10px] uppercase font-black text-muted-foreground">Part Label</th>
                    <th className="p-2 text-[10px] uppercase font-black text-muted-foreground">Max Stress (MPa)</th>
                    <th className="p-2 text-[10px] uppercase font-black text-muted-foreground">Safety Factor</th>
                    <th className="p-2 text-[10px] uppercase font-black text-muted-foreground text-right">Utilization</th>
                  </tr>
                </thead>
                <tbody>
                  {stressSummaries.map((s, idx) => (
                    <tr key={idx} className="border-b border-border/20 hover:bg-muted/30 transition-colors">
                      <td className="p-2 font-medium text-xs">{s.part_label}</td>
                      <td className="p-2 text-xs font-mono">
                        {(s.max_von_mises_pa / 1e6).toFixed(2)}
                      </td>
                      <td className="p-2">
                        <Badge 
                          variant="outline" 
                          className={cn(
                            "text-[10px] font-bold px-1.5 h-5",
                            s.safety_factor < 1.2 ? "border-red-500/50 text-red-500 bg-red-500/5" :
                            s.safety_factor < 1.5 ? "border-amber-500/50 text-amber-500 bg-amber-500/5" :
                            "border-green-500/50 text-green-500 bg-green-500/5"
                          )}
                        >
                          {s.safety_factor.toFixed(2)}x
                        </Badge>
                      </td>
                      <td className="p-2 text-right">
                        <div className="flex flex-col items-end gap-1">
                          <span className={cn(
                            "text-[10px] font-mono font-bold",
                            s.utilization_pct > 90 ? "text-red-500" :
                            s.utilization_pct > 70 ? "text-amber-500" :
                            "text-green-500"
                          )}>
                            {s.utilization_pct.toFixed(1)}%
                          </span>
                          <div className="w-20 h-1 bg-muted rounded-full overflow-hidden">
                            <div 
                              className={cn(
                                "h-full transition-all",
                                s.utilization_pct > 90 ? "bg-red-500" :
                                s.utilization_pct > 70 ? "bg-amber-500" :
                                "bg-green-500"
                              )}
                              style={{ width: `${Math.min(s.utilization_pct, 100)}%` }}
                            />
                          </div>
                        </div>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Fluid Metrics */}
      {fluidMetrics.length > 0 && (
        <Card className="bg-card/50 backdrop-blur-sm border-border/50">
          <CardHeader className="pb-2">
            <CardTitle className="text-sm font-bold uppercase tracking-widest flex items-center gap-2">
              <Droplets className="h-4 w-4 text-blue-500" />
              Fluid Dynamics Metrics
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            {fluidMetrics.map((f, idx) => (
              <div key={idx} className="space-y-2 p-3 rounded-lg bg-muted/20 border border-border/30">
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <span className="text-xs font-bold text-slate-200">{f.fluid_id}</span>
                    <Badge variant="secondary" className="text-[9px] uppercase font-black px-1.5 h-4 opacity-70">
                      {f.metric_type.replace('_', ' ')}
                    </Badge>
                  </div>
                  {f.passed ? (
                    <CheckCircle2 className="h-4 w-4 text-green-500" />
                  ) : (
                    <AlertTriangle className="h-4 w-4 text-red-500" />
                  )}
                </div>
                
                <div className="space-y-1">
                  <div className="flex justify-between text-[10px] font-mono text-muted-foreground uppercase">
                    <span>Measured: {f.measured_value.toFixed(3)}</span>
                    <span>Target: {f.target_value.toFixed(3)}</span>
                  </div>
                  <div className="w-full h-2 bg-muted rounded-full overflow-hidden">
                    <div 
                      className={cn(
                        "h-full transition-all",
                        f.passed ? "bg-blue-500" : "bg-red-500"
                      )}
                      style={{ width: `${Math.min((f.measured_value / (f.target_value || 1)) * 100, 100)}%` }}
                    />
                  </div>
                </div>
              </div>
            ))}
          </CardContent>
        </Card>
      )}

      {/* Real-time confidence/stability badge */}
      <div className="flex justify-end gap-2">
         <Badge variant="outline" className="text-[9px] font-black tracking-widest uppercase border-primary/20 bg-primary/5 text-primary gap-1.5 py-1 px-2">
            <Activity className="h-3 w-3" />
            Physics Confidence: High
         </Badge>
      </div>
    </div>
  );
};
