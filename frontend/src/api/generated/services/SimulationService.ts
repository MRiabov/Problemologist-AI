/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { RunSimulationRequest } from '../models/RunSimulationRequest';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class SimulationService {
    /**
     * Run Simulation
     * Trigger a benchmark generation session (simulation pipeline).
     * This endpoint reuses the benchmark generation graph but is called 'simulation'
     * in the frontend for historical reasons.
     * @param requestBody
     * @returns any Successful Response
     * @throws ApiError
     */
    public static runSimulationApiSimulationRunPost(
        requestBody: RunSimulationRequest,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/simulation/run',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
