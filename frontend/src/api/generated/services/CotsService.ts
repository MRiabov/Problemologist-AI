/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CotsMetadataResponse } from '../models/CotsMetadataResponse';
import type { CotsSearchItem } from '../models/CotsSearchItem';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class CotsService {
    /**
     * Search Cots
     * Search for COTS parts by name or category.
     * @param q Search query
     * @param limit
     * @returns CotsSearchItem Successful Response
     * @throws ApiError
     */
    public static searchCotsApiCotsSearchGet(
        q: string,
        limit: number = 20,
    ): CancelablePromise<Array<CotsSearchItem>> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/cots/search',
            query: {
                'q': q,
                'limit': limit,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Catalog Metadata
     * Get the catalog metadata (version, commit, etc.).
     * @returns CotsMetadataResponse Successful Response
     * @throws ApiError
     */
    public static getCatalogMetadataApiCotsMetadataGet(): CancelablePromise<CotsMetadataResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/cots/metadata',
        });
    }
}
