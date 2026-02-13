/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { TraceType } from './TraceType';
export type TraceResponse = {
    id: number;
    langfuse_trace_id: (string | null);
    trace_type: TraceType;
    name: (string | null);
    content: (string | null);
    metadata_vars?: (Record<string, any> | null);
    feedback_score?: (number | null);
    feedback_comment?: (string | null);
    created_at: string;
};

