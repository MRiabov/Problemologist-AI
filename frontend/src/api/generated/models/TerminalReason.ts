/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * Reason why an episode ended in a terminal state.
 */
export enum TerminalReason {
    APPROVED = 'APPROVED',
    REJECTED_BY_REVIEW = 'REJECTED_BY_REVIEW',
    HANDOFF_INVARIANT_VIOLATION = 'HANDOFF_INVARIANT_VIOLATION',
    OUT_OF_TURN_BUDGET = 'OUT_OF_TURN_BUDGET',
    OUT_OF_TOKEN_BUDGET = 'OUT_OF_TOKEN_BUDGET',
    TIMEOUT = 'TIMEOUT',
    INTERNAL_ERROR = 'INTERNAL_ERROR',
    SYSTEM_TOOL_RETRY_EXHAUSTED = 'SYSTEM_TOOL_RETRY_EXHAUSTED',
    INFRA_ERROR = 'INFRA_ERROR',
    USER_CANCELLED = 'USER_CANCELLED',
}
