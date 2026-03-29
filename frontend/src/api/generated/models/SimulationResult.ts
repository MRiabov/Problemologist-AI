/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { FailureReason } from './FailureReason';
import type { FluidMetricResult } from './FluidMetricResult';
import type { SimulationFailure } from './SimulationFailure';
import type { SimulationRenderProvenance } from './SimulationRenderProvenance';
import type { StressFieldData } from './StressFieldData';
import type { StressSummary } from './StressSummary';
export type SimulationResult = {
    success: boolean;
    summary: string;
    failure_reason?: (string | null);
    fail_mode?: (FailureReason | null);
    failure?: (SimulationFailure | null);
    render_provenance?: (SimulationRenderProvenance | null);
    render_paths?: Array<string>;
    mjcf_content?: (string | null);
    stress_summaries?: Array<StressSummary>;
    stress_fields?: Record<string, StressFieldData>;
    fluid_metrics?: Array<FluidMetricResult>;
    total_cost?: number;
    total_weight_g?: number;
    confidence?: SimulationResult.confidence;
};
export namespace SimulationResult {
    export enum confidence {
        LOW = 'low',
        MEDIUM = 'medium',
        HIGH = 'high',
        APPROXIMATE = 'approximate',
    }
}

