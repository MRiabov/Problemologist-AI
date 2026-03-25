/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { ReviewDecision } from './ReviewDecision';
import type { SimulatorBackendType } from './SimulatorBackendType';
/**
 * Structured metadata for individual traces (tool calls, nodes, etc.).
 */
export type TraceMetadata = {
    tool_name?: (string | null);
    tool_args?: (Record<string, any> | null);
    observation?: (string | null);
    prediction?: (string | null);
    error?: (string | null);
    simulation_run_id?: (string | null);
    backend?: (SimulatorBackendType | null);
    motor_states?: Record<string, string>;
    cots_query_id?: (string | null);
    review_id?: (string | null);
    decision?: (ReviewDecision | null);
    checklist?: Record<string, (string | number | boolean)>;
    reasoning_step_index?: (number | null);
    reasoning_source?: (string | null);
    additional_info?: Record<string, any>;
};

