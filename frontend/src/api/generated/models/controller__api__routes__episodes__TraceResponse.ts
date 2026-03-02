/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { TraceType } from './TraceType';
export type controller__api__routes__episodes__TraceResponse = {
    id: number;
    langfuse_trace_id: (string | null);
    simulation_run_id?: (string | null);
    cots_query_id?: (string | null);
    review_id?: (string | null);
    trace_type: TraceType;
    name: (string | null);
    content: (string | null);
    metadata_vars?: (Record<string, any> | null);
    feedback_score?: (number | null);
    feedback_comment?: (string | null);
    created_at: string;
};

