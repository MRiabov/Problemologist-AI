/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class CotsService {
    /**
     * Search Cots
     * Search for COTS parts by name or category.
     * @param q Search query
     * @param limit
     * @returns any Successful Response
     * @throws ApiError
     */
    public static searchCotsCotsSearchGet(
        q: string,
        limit: number = 20,
    ): CancelablePromise<Array<Record<string, any>>> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/cots/search',
            query: {
                'q': q,
                'limit': limit,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
