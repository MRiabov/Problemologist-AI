/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BenchmarkAttachmentPolicySummary } from './BenchmarkAttachmentPolicySummary';
/**
 * Persisted handoff manifest used to gate reviewer entry.
 */
export type ReviewManifest = {
    status: string;
    reviewer_stage: ReviewManifest.reviewer_stage;
    timestamp?: (string | null);
    session_id: string;
    revision?: (string | null);
    episode_id?: (string | null);
    worker_session_id?: (string | null);
    benchmark_episode_id?: (string | null);
    benchmark_worker_session_id?: (string | null);
    benchmark_revision?: (string | null);
    solution_revision?: (string | null);
    environment_version?: (string | null);
    preview_evidence_paths?: Array<string>;
    script_path: string;
    script_sha256: string;
    validation_success: boolean;
    validation_timestamp: number;
    simulation_success: boolean;
    simulation_summary: string;
    simulation_timestamp: number;
    motion_evidence_verified?: (boolean | null);
    goal_reached?: (boolean | null);
    renders?: Array<string>;
    benchmark_attachment_policy_summary?: Array<BenchmarkAttachmentPolicySummary>;
    mjcf_path?: (string | null);
    cad_path?: (string | null);
    objectives_path?: (string | null);
    assembly_definition_path?: (string | null);
};
export namespace ReviewManifest {
    export enum reviewer_stage {
        BENCHMARK_REVIEWER = 'benchmark_reviewer',
        ENGINEERING_EXECUTION_REVIEWER = 'engineering_execution_reviewer',
        ELECTRONICS_REVIEWER = 'electronics_reviewer',
    }
}

