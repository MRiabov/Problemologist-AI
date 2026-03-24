/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { SimulationMetrics } from './SimulationMetrics';
/**
 * Result of batched runtime-randomization verification.
 */
export type MultiRunResult = {
    num_scenes: number;
    success_count: number;
    success_rate: number;
    is_consistent: boolean;
    individual_results: Array<SimulationMetrics>;
    fail_reasons: Array<string>;
    scene_build_count?: number;
    backend_run_count?: number;
    batched_execution?: boolean;
};

