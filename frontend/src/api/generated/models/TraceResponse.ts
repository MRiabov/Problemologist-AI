/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { TraceMetadata } from './TraceMetadata';
import type { TraceType } from './TraceType';
export type TraceResponse = {
    id: number;
    user_session_id?: (string | null);
    langfuse_trace_id: (string | null);
    simulation_run_id?: (string | null);
    cots_query_id?: (string | null);
    review_id?: (string | null);
    trace_type: TraceType;
    name: (string | null);
    content: (string | null);
    metadata_vars?: (TraceMetadata | null);
    feedback_score?: (number | null);
    feedback_comment?: (string | null);
    created_at: string;
};

