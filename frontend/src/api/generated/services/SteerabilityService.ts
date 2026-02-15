/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { SteerablePrompt_Input } from '../models/SteerablePrompt_Input';
import type { SteerablePrompt_Output } from '../models/SteerablePrompt_Output';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class SteerabilityService {
    /**
     * Steer Agent
     * Enqueue a steered prompt for the agent.
     * If the agent is idle, it will be picked up immediately (implemented in WP04).
     * @param sessionId The agent session ID
     * @param requestBody
     * @returns any Successful Response
     * @throws ApiError
     */
    public static steerAgentApiV1SessionsSessionIdSteerPost(
        sessionId: string,
        requestBody: SteerablePrompt_Input,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/v1/sessions/{session_id}/steer',
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
     * Get Steering Queue
     * Return all currently queued prompts for this session.
     * @param sessionId The agent session ID
     * @returns SteerablePrompt_Output Successful Response
     * @throws ApiError
     */
    public static getSteeringQueueApiV1SessionsSessionIdQueueGet(
        sessionId: string,
    ): CancelablePromise<Array<SteerablePrompt_Output>> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/v1/sessions/{session_id}/queue',
            path: {
                'session_id': sessionId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
