/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BaseEvent } from './BaseEvent';
import type { SimulationArtifacts } from './SimulationArtifacts';
/**
 * Response from a benchmark tool.
 */
export type BenchmarkToolResponse = {
    success: boolean;
    message: string;
    confidence?: string;
    artifacts?: (SimulationArtifacts | null);
    events?: Array<BaseEvent>;
};

