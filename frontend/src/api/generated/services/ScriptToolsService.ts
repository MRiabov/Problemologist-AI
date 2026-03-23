/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BenchmarkToolResponse } from '../models/BenchmarkToolResponse';
import type { ScriptToolRequest } from '../models/ScriptToolRequest';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class ScriptToolsService {
    /**
     * Validate Script
     * @param xSessionId
     * @param requestBody
     * @returns BenchmarkToolResponse Successful Response
     * @throws ApiError
     */
    public static validateScriptApiScriptToolsValidatePost(
        xSessionId: string,
        requestBody: ScriptToolRequest,
    ): CancelablePromise<BenchmarkToolResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/script-tools/validate',
            headers: {
                'x-session-id': xSessionId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Simulate Script
     * @param xSessionId
     * @param requestBody
     * @returns BenchmarkToolResponse Successful Response
     * @throws ApiError
     */
    public static simulateScriptApiScriptToolsSimulatePost(
        xSessionId: string,
        requestBody: ScriptToolRequest,
    ): CancelablePromise<BenchmarkToolResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/script-tools/simulate',
            headers: {
                'x-session-id': xSessionId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Verify Script
     * @param xSessionId
     * @param requestBody
     * @returns BenchmarkToolResponse Successful Response
     * @throws ApiError
     */
    public static verifyScriptApiScriptToolsVerifyPost(
        xSessionId: string,
        requestBody: ScriptToolRequest,
    ): CancelablePromise<BenchmarkToolResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/script-tools/verify',
            headers: {
                'x-session-id': xSessionId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Submit Script
     * @param xSessionId
     * @param requestBody
     * @returns BenchmarkToolResponse Successful Response
     * @throws ApiError
     */
    public static submitScriptApiScriptToolsSubmitPost(
        xSessionId: string,
        requestBody: ScriptToolRequest,
    ): CancelablePromise<BenchmarkToolResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/script-tools/submit',
            headers: {
                'x-session-id': xSessionId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Validate Script
     * @param xSessionId
     * @param requestBody
     * @returns BenchmarkToolResponse Successful Response
     * @throws ApiError
     */
    public static validateScriptScriptToolsValidatePost(
        xSessionId: string,
        requestBody: ScriptToolRequest,
    ): CancelablePromise<BenchmarkToolResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/script-tools/validate',
            headers: {
                'x-session-id': xSessionId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Simulate Script
     * @param xSessionId
     * @param requestBody
     * @returns BenchmarkToolResponse Successful Response
     * @throws ApiError
     */
    public static simulateScriptScriptToolsSimulatePost(
        xSessionId: string,
        requestBody: ScriptToolRequest,
    ): CancelablePromise<BenchmarkToolResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/script-tools/simulate',
            headers: {
                'x-session-id': xSessionId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Verify Script
     * @param xSessionId
     * @param requestBody
     * @returns BenchmarkToolResponse Successful Response
     * @throws ApiError
     */
    public static verifyScriptScriptToolsVerifyPost(
        xSessionId: string,
        requestBody: ScriptToolRequest,
    ): CancelablePromise<BenchmarkToolResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/script-tools/verify',
            headers: {
                'x-session-id': xSessionId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Submit Script
     * @param xSessionId
     * @param requestBody
     * @returns BenchmarkToolResponse Successful Response
     * @throws ApiError
     */
    public static submitScriptScriptToolsSubmitPost(
        xSessionId: string,
        requestBody: ScriptToolRequest,
    ): CancelablePromise<BenchmarkToolResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/script-tools/submit',
            headers: {
                'x-session-id': xSessionId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
