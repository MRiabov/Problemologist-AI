/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BenchmarkGenerateRequest } from '../models/BenchmarkGenerateRequest';
import type { UpdateObjectivesRequest } from '../models/UpdateObjectivesRequest';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class BenchmarkService {
    /**
     * Generate Benchmark
     * Trigger a new benchmark generation session.
     * @param requestBody
     * @returns any Successful Response
     * @throws ApiError
     */
    public static generateBenchmarkBenchmarkGeneratePost(
        requestBody: BenchmarkGenerateRequest,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/benchmark/generate',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Confirm Benchmark
     * Confirm and continue benchmark generation after planning.
     * @param sessionId
     * @returns any Successful Response
     * @throws ApiError
     */
    public static confirmBenchmarkBenchmarkSessionIdConfirmPost(
        sessionId: string,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/benchmark/{session_id}/confirm',
            path: {
                'session_id': sessionId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Session
     * @param sessionId
     * @returns any Successful Response
     * @throws ApiError
     */
    public static getSessionBenchmarkSessionIdGet(
        sessionId: string,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/benchmark/{session_id}',
            path: {
                'session_id': sessionId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Update Objectives
     * Update objectives.yaml for a specific session.
     * @param sessionId
     * @param requestBody
     * @returns any Successful Response
     * @throws ApiError
     */
    public static updateObjectivesBenchmarkSessionIdObjectivesPost(
        sessionId: string,
        requestBody: UpdateObjectivesRequest,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/benchmark/{session_id}/objectives',
            path: {
                'session_id': sessionId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
