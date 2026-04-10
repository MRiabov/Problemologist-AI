/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * Response from enqueuing or starting a steerability prompt.
 */
export type SteerabilityResponse = {
    status: SteerabilityResponse.status;
    queue_position: number;
};
export namespace SteerabilityResponse {
    export enum status {
        QUEUED = 'queued',
        STARTED = 'started',
    }
}

