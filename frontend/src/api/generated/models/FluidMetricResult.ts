/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { FluidObjectiveType } from './FluidObjectiveType';
export type FluidMetricResult = {
    metric_type: FluidObjectiveType;
    fluid_id: string;
    measured_value: number;
    target_value: number;
    passed: boolean;
};

