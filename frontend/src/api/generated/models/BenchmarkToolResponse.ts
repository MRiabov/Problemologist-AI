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
    confidence?: BenchmarkToolResponse.confidence;
    artifacts?: (SimulationArtifacts | null);
    events?: Array<BaseEvent>;
};
export namespace BenchmarkToolResponse {
    export enum confidence {
        LOW = 'low',
        MEDIUM = 'medium',
        HIGH = 'high',
        APPROXIMATE = 'approximate',
    }
}

