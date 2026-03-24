/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { FailureReason } from './FailureReason';
import type { FluidMetricResult } from './FluidMetricResult';
import type { SimulationFailure } from './SimulationFailure';
import type { StressFieldData } from './StressFieldData';
import type { StressSummary } from './StressSummary';
export type SimulationMetrics = {
    total_time?: number;
    total_energy?: number;
    max_velocity?: number;
    max_stress?: number;
    success?: boolean;
    fail_reason?: (string | null);
    fail_mode?: (FailureReason | null);
    failure?: (SimulationFailure | null);
    stress_summaries?: Array<StressSummary>;
    stress_fields?: Record<string, StressFieldData>;
    fluid_metrics?: Array<FluidMetricResult>;
    events?: Array<Record<string, any>>;
    confidence?: SimulationMetrics.confidence;
};
export namespace SimulationMetrics {
    export enum confidence {
        LOW = 'low',
        MEDIUM = 'medium',
        HIGH = 'high',
        APPROXIMATE = 'approximate',
    }
}

