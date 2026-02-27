/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BenchmarkConfirmResponse } from '../models/BenchmarkConfirmResponse';
import type { BenchmarkGenerateRequest } from '../models/BenchmarkGenerateRequest';
import type { BenchmarkGenerateResponse } from '../models/BenchmarkGenerateResponse';
import type { BenchmarkObjectivesResponse } from '../models/BenchmarkObjectivesResponse';
import type { ConfirmRequest } from '../models/ConfirmRequest';
import type { controller__api__schemas__EpisodeResponse } from '../models/controller__api__schemas__EpisodeResponse';
import type { UpdateObjectivesRequest } from '../models/UpdateObjectivesRequest';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class BenchmarkService {
    /**
     * Generate Benchmark
     * Trigger a new benchmark generation session.
     * @param requestBody
     * @returns BenchmarkGenerateResponse Successful Response
     * @throws ApiError
     */
    public static generateBenchmarkApiBenchmarkGeneratePost(
        requestBody: BenchmarkGenerateRequest,
    ): CancelablePromise<BenchmarkGenerateResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/benchmark/generate',
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
     * @param requestBody
     * @returns BenchmarkConfirmResponse Successful Response
     * @throws ApiError
     */
    public static confirmBenchmarkApiBenchmarkSessionIdConfirmPost(
        sessionId: string,
        requestBody: ConfirmRequest,
    ): CancelablePromise<BenchmarkConfirmResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/benchmark/{session_id}/confirm',
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
    /**
     * Get Session
     * @param sessionId
     * @returns controller__api__schemas__EpisodeResponse Successful Response
     * @throws ApiError
     */
    public static getSessionApiBenchmarkSessionIdGet(
        sessionId: string,
    ): CancelablePromise<controller__api__schemas__EpisodeResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/benchmark/{session_id}',
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
     * @returns BenchmarkObjectivesResponse Successful Response
     * @throws ApiError
     */
    public static updateObjectivesApiBenchmarkSessionIdObjectivesPost(
        sessionId: string,
        requestBody: UpdateObjectivesRequest,
    ): CancelablePromise<BenchmarkObjectivesResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/benchmark/{session_id}/objectives',
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
