import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import type { MultiRunResult } from "../../api/generated/models/MultiRunResult";

export interface VerificationSceneSummary {
  index: number;
  passed: boolean;
  totalTime: number | null;
  maxVelocity: number | null;
  failReason: string | null;
  failMode: string | null;
  confidence: string | null;
}

export interface VerificationSummary {
  validationResultsPath: string;
  numScenes: number;
  successCount: number;
  successRate: number;
  isConsistent: boolean;
  failReasons: string[];
  sceneBuildCount: number;
  backendRunCount: number;
  batchedExecution: boolean;
  scenes: VerificationSceneSummary[];
}

const VALIDATION_RESULTS_NAME = "validation_results.json";

function safeNumber(value: unknown): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

export function getValidationResultsAsset(
  assets: AssetResponse[] = [],
): AssetResponse | null {
  return (
    assets.find((asset) =>
      asset.s3_path.toLowerCase().endsWith(VALIDATION_RESULTS_NAME),
    ) ?? null
  );
}

export function parseVerificationResultFromAssets(
  assets: AssetResponse[] = [],
): MultiRunResult | null {
  const validationAsset = getValidationResultsAsset(assets);
  if (!validationAsset?.content) {
    return null;
  }

  try {
    const parsed = JSON.parse(validationAsset.content) as {
      verification_result?: MultiRunResult | null;
    };
    return parsed.verification_result ?? null;
  } catch {
    return null;
  }
}

export function summarizeVerificationResult(
  verificationResult: MultiRunResult | null | undefined,
  validationResultsPath = VALIDATION_RESULTS_NAME,
): VerificationSummary | null {
  if (!verificationResult) {
    return null;
  }

  const scenes: VerificationSceneSummary[] = (
    verificationResult.individual_results ?? []
  ).map((scene, index) => ({
    index: index + 1,
    passed: !!scene.success,
    totalTime: safeNumber(scene.total_time),
    maxVelocity: safeNumber(scene.max_velocity),
    failReason: scene.fail_reason ?? scene.failure?.detail ?? null,
    failMode: scene.fail_mode ?? null,
    confidence: scene.confidence ?? null,
  }));

  return {
    validationResultsPath,
    numScenes: verificationResult.num_scenes,
    successCount: verificationResult.success_count,
    successRate: verificationResult.success_rate,
    isConsistent: verificationResult.is_consistent,
    failReasons: verificationResult.fail_reasons ?? [],
    sceneBuildCount: verificationResult.scene_build_count ?? 1,
    backendRunCount: verificationResult.backend_run_count ?? 1,
    batchedExecution: verificationResult.batched_execution ?? true,
    scenes,
  };
}

export function summarizeVerificationAssets(
  assets: AssetResponse[] = [],
): VerificationSummary | null {
  const validationAsset = getValidationResultsAsset(assets);
  const verificationResult = parseVerificationResultFromAssets(assets);
  return summarizeVerificationResult(
    verificationResult,
    validationAsset?.s3_path ?? VALIDATION_RESULTS_NAME,
  );
}

export function formatVerificationSummaryLines(
  summary: VerificationSummary | null | undefined,
): string[] {
  if (!summary) {
    return [];
  }

  const lines: string[] = [
    `validation_results.json: ${summary.validationResultsPath}`,
    `num_scenes: ${summary.numScenes}`,
    `success_count: ${summary.successCount}`,
    `success_rate: ${summary.successRate.toFixed(2)}`,
    `is_consistent: ${summary.isConsistent ? "true" : "false"}`,
    `scene_build_count: ${summary.sceneBuildCount}`,
    `backend_run_count: ${summary.backendRunCount}`,
    `batched_execution: ${summary.batchedExecution ? "true" : "false"}`,
  ];

  if (summary.failReasons.length > 0) {
    lines.push("fail_reasons:");
    summary.failReasons.forEach((reason) => {
      lines.push(`  - ${reason}`);
    });
  }

  summary.scenes.forEach((scene) => {
    lines.push(
      `scene_${scene.index}: ${scene.passed ? "pass" : "fail"}${scene.failReason ? ` (${scene.failReason})` : ""}`,
    );
  });

  return lines;
}
