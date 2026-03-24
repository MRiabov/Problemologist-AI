/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * How a seed linkage was determined for an episode/dataset row.
 */
export enum SeedMatchMethod {
    RUNTIME_EXPLICIT = 'runtime_explicit',
    EXACT_TASK = 'exact_task',
    NO_EXACT_TASK_MATCH = 'no_exact_task_match',
    AMBIGUOUS_EXACT_TASK = 'ambiguous_exact_task',
}
