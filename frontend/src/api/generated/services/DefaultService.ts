/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AgentRunRequest } from '../models/AgentRunRequest';
import type { AgentRunResponse } from '../models/AgentRunResponse';
import type { EpisodeCreateResponse } from '../models/EpisodeCreateResponse';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class DefaultService {
    /**
     * Read Root
     * Root endpoint for service discovery.
     * @returns any Successful Response
     * @throws ApiError
     */
    public static readRootGet(): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/',
        });
    }
    /**
     * Create Test Episode
     * Create a dummy episode for testing purposes (no agent run).
     * @param requestBody
     * @returns EpisodeCreateResponse Successful Response
     * @throws ApiError
     */
    public static createTestEpisodeTestEpisodesPost(
        requestBody: AgentRunRequest,
    ): CancelablePromise<EpisodeCreateResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/test/episodes',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Health Check
     * Health check endpoint.
     * @returns any Successful Response
     * @throws ApiError
     */
    public static healthCheckHealthGet(): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/health',
        });
    }
    /**
     * Run Agent
     * @param requestBody
     * @returns AgentRunResponse Successful Response
     * @throws ApiError
     */
    public static runAgentAgentRunPost(
        requestBody: AgentRunRequest,
    ): CancelablePromise<AgentRunResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/agent/run',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
