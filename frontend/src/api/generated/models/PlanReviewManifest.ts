/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AgentName } from './AgentName';
/**
 * Planner handoff manifest used to gate engineering plan reviewer entry.
 */
export type PlanReviewManifest = {
    status: string;
    reviewer_stage: PlanReviewManifest.reviewer_stage;
    session_id: string;
    planner_node_type: AgentName;
    episode_id?: (string | null);
    worker_session_id?: (string | null);
    benchmark_revision?: (string | null);
    environment_version?: (string | null);
    artifact_hashes: Record<string, string>;
};
export namespace PlanReviewManifest {
    export enum reviewer_stage {
        BENCHMARK_PLAN_REVIEWER = 'benchmark_plan_reviewer',
        ENGINEER_PLAN_REVIEWER = 'engineer_plan_reviewer',
    }
}

