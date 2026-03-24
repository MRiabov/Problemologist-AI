import * as yaml from "js-yaml";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import { ReviewDecision } from "../../api/generated/models/ReviewDecision";
import type { MultiRunResult } from "../../api/generated/models/MultiRunResult";

export interface ValidationResultsRecord {
  success: boolean;
  message?: string | null;
  timestamp?: number | null;
  script_path?: string | null;
  script_sha256?: string | null;
  verification_result?: MultiRunResult | null;
}

export interface StabilitySceneSummary {
  sceneIndex: number;
  success: boolean;
  summary: string;
  failReason: string | null;
  failureMode: string | null;
}

export interface StabilitySummary {
  batchWidth: number;
  successCount: number;
  successRate: number;
  isConsistent: boolean;
  sceneBuildCount: number;
  backendRunCount: number;
  batchedExecution: boolean;
  sceneSummaries: StabilitySceneSummary[];
}

export interface ReviewContentOptions {
  decision: ReviewDecision;
  reason: string;
  stabilitySummary: StabilitySummary;
  context?: {
    benchmarkId?: string | null;
    episodeId?: string | null;
    priorEpisodeId?: string | null;
    revisionCount?: number | null;
    reused?: boolean | null;
  };
}

export function getValidationResultsAsset(
  assets: AssetResponse[] | null | undefined,
): AssetResponse | null {
  return (
    assets?.find(
      (asset) => asset.s3_path.toLowerCase().endsWith("validation_results.json"),
    ) ?? null
  );
}

export function parseValidationResults(
  content: string | null | undefined,
): ValidationResultsRecord | null {
  if (!content) {
    return null;
  }

  try {
    const data = JSON.parse(content) as ValidationResultsRecord;
    if (!data || typeof data !== "object") {
      return null;
    }
    return data;
  } catch {
    return null;
  }
}

export function getValidationResultsRecord(
  assets: AssetResponse[] | null | undefined,
): ValidationResultsRecord | null {
  const asset = getValidationResultsAsset(assets);
  return parseValidationResults(asset?.content);
}

export function getVerificationResult(
  assets: AssetResponse[] | null | undefined,
): MultiRunResult | null {
  return getValidationResultsRecord(assets)?.verification_result ?? null;
}

export function summarizeVerificationResult(
  result: MultiRunResult | null | undefined,
): StabilitySummary | null {
  if (!result) {
    return null;
  }

  return {
    batchWidth: result.num_scenes,
    successCount: result.success_count,
    successRate: result.success_rate,
    isConsistent: result.is_consistent,
    sceneBuildCount: result.scene_build_count ?? 1,
    backendRunCount: result.backend_run_count ?? 1,
    batchedExecution: result.batched_execution ?? true,
    sceneSummaries: result.individual_results.map((scene, index) => {
      const failReason =
        scene.fail_reason ??
        scene.failure?.detail ??
        (scene.failure ? String(scene.failure.reason) : null);
      const failureMode = scene.fail_mode ?? scene.failure?.reason ?? null;
      return {
        sceneIndex: index + 1,
        success: !!scene.success,
        summary: scene.success
          ? `Scene ${index + 1}: pass`
          : `Scene ${index + 1}: ${failReason ?? "failed"}`,
        failReason,
        failureMode:
          typeof failureMode === "string"
            ? failureMode
            : failureMode
              ? String(failureMode)
              : null,
      };
    }),
  };
}

export function formatStabilitySummaryLines(
  summary: StabilitySummary | null,
): string[] {
  if (!summary) {
    return ["Stability summary unavailable."];
  }

  const failureCount = summary.sceneSummaries.filter((scene) => !scene.success).length;
  const passCount = summary.sceneSummaries.length - failureCount;
  return [
    `Batch width: ${summary.batchWidth}`,
    `Success count: ${summary.successCount} / ${summary.batchWidth}`,
    `Success rate: ${(summary.successRate * 100).toFixed(1)}%`,
    `Consistency: ${summary.isConsistent ? "consistent" : "inconsistent"}`,
    `Batched execution: ${summary.batchedExecution ? "yes" : "no"}`,
    `Scene passes: ${passCount}`,
    `Scene failures: ${failureCount}`,
  ];
}

export function buildReviewContent({
  decision,
  reason,
  stabilitySummary,
  context,
}: ReviewContentOptions): string {
  const comments = reason.trim() ? [reason.trim()] : [];
  const frontmatter = {
    decision:
      decision === ReviewDecision.APPROVED
        ? "approved"
        : decision === ReviewDecision.REJECTED
          ? "rejected"
          : decision.toLowerCase(),
    comments,
    evidence: {
      stability_summary: stabilitySummary,
      stability_summary_source: "validation_results.json",
      review_context: context ?? {},
    },
  };

  const body = [
    "# Peer Review",
    "",
    `Decision: ${frontmatter.decision.toUpperCase()}`,
    "",
    "## Stability Summary",
    "Stability summary:",
    "- Source artifact: validation_results.json",
    ...formatStabilitySummaryLines(stabilitySummary).map((line) => `- ${line}`),
  ];

  if (context) {
    body.push(
      "",
      "## Review Context",
      `- Benchmark ID: ${context.benchmarkId ?? "n/a"}`,
      `- Episode ID: ${context.episodeId ?? "n/a"}`,
      `- Parent Episode ID: ${context.priorEpisodeId ?? "n/a"}`,
      `- Revision Count: ${context.revisionCount ?? "n/a"}`,
      `- Reused: ${context.reused == null ? "n/a" : context.reused ? "true" : "false"}`,
    );
  }

  return `---\n${yaml.dump(frontmatter, {
    noRefs: true,
    sortKeys: false,
    lineWidth: 120,
  }).trim()}\n---\n${body.join("\n")}\n`;
}
