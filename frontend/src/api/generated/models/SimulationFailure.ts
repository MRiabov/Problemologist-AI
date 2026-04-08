/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { FailureReason } from './FailureReason';
import type { PayloadTrajectoryMonitorState } from './PayloadTrajectoryMonitorState';
/**
 * Structured failure information for simulation.
 */
export type SimulationFailure = {
    reason: FailureReason;
    detail?: (string | null);
    payload_trajectory_monitor?: (PayloadTrajectoryMonitorState | null);
};

