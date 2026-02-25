/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { SimulatorBackendType } from './SimulatorBackendType';
export type BenchmarkGenerateRequest = {
    prompt: string;
    max_cost?: (number | null);
    max_weight?: (number | null);
    target_quantity?: (number | null);
    backend?: SimulatorBackendType;
};

