/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BaseEvent } from './BaseEvent';
import type { SimulationArtifacts } from './SimulationArtifacts';
import type { SimulationConfidence } from './SimulationConfidence';
/**
 * Response from a benchmark tool.
 */
export type BenchmarkToolResponse = {
    success: boolean;
    message: string;
    confidence?: SimulationConfidence;
    artifacts?: (SimulationArtifacts | null);
    events?: Array<BaseEvent>;
};

