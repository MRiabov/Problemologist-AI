/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AssetResponse } from './AssetResponse';
import type { EntryValidationContext } from './EntryValidationContext';
import type { EpisodeMetadata } from './EpisodeMetadata';
import type { EpisodeStatus } from './EpisodeStatus';
import type { FailureClass } from './FailureClass';
import type { ReplayArtifactRecord } from './ReplayArtifactRecord';
import type { ReplayFailureSignal } from './ReplayFailureSignal';
import type { ReplayReviewManifestResponse } from './ReplayReviewManifestResponse';
import type { ReplayTraceIds } from './ReplayTraceIds';
import type { ReviewDecisionEvent } from './ReviewDecisionEvent';
import type { SimulationResult } from './SimulationResult';
import type { TerminalReason } from './TerminalReason';
import type { TraceResponse } from './TraceResponse';
import type { ValidationResultRecord } from './ValidationResultRecord';
export type EpisodeReplayResponse = {
    id: string;
    user_session_id?: (string | null);
    task: string;
    status: EpisodeStatus;
    created_at: string;
    updated_at: string;
    skill_git_hash?: (string | null);
    template_versions?: (Record<string, any> | null);
    metadata_vars?: (EpisodeMetadata | null);
    todo_list?: (Record<string, any> | null);
    journal?: (string | null);
    plan?: (string | null);
    validation_logs?: (Array<string> | null);
    last_trace_id?: (number | null);
    traces?: Array<TraceResponse>;
    assets?: Array<AssetResponse>;
    worker_session_id?: (string | null);
    simulation_run_id?: (string | null);
    cots_query_id?: (string | null);
    review_id?: (string | null);
    terminal_reason?: (TerminalReason | null);
    failure_class?: (FailureClass | null);
    detailed_status?: (string | null);
    entry_validation?: (EntryValidationContext | null);
    replay_artifacts?: Array<ReplayArtifactRecord>;
    trace_ids?: ReplayTraceIds;
    validation_result?: (ValidationResultRecord | null);
    simulation_result?: (SimulationResult | null);
    review_manifests?: Array<ReplayReviewManifestResponse>;
    review_decision_events?: Array<ReviewDecisionEvent>;
    failure_signals?: Array<ReplayFailureSignal>;
};

