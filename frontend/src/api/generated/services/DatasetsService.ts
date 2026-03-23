/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { DatasetExportRequest } from '../models/DatasetExportRequest';
import type { DatasetExportResponse } from '../models/DatasetExportResponse';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class DatasetsService {
    /**
     * Export Dataset
     * @param requestBody
     * @returns DatasetExportResponse Successful Response
     * @throws ApiError
     */
    public static exportDatasetApiDatasetsExportPost(
        requestBody: DatasetExportRequest,
    ): CancelablePromise<DatasetExportResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/datasets/export',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Export Dataset For Episode
     * @param episodeId
     * @returns DatasetExportResponse Successful Response
     * @throws ApiError
     */
    public static exportDatasetForEpisodeApiDatasetsEpisodeIdExportPost(
        episodeId: string,
    ): CancelablePromise<DatasetExportResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/api/datasets/{episode_id}/export',
            path: {
                'episode_id': episodeId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Dataset Export
     * @param exportId
     * @returns DatasetExportResponse Successful Response
     * @throws ApiError
     */
    public static getDatasetExportApiDatasetsExportsExportIdGet(
        exportId: string,
    ): CancelablePromise<DatasetExportResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/datasets/exports/{export_id}',
            path: {
                'export_id': exportId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Dataset Export By Episode
     * @param episodeId
     * @returns DatasetExportResponse Successful Response
     * @throws ApiError
     */
    public static getDatasetExportByEpisodeApiDatasetsEpisodesEpisodeIdGet(
        episodeId: string,
    ): CancelablePromise<DatasetExportResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/api/datasets/episodes/{episode_id}',
            path: {
                'episode_id': episodeId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Export Dataset
     * @param requestBody
     * @returns DatasetExportResponse Successful Response
     * @throws ApiError
     */
    public static exportDatasetDatasetsExportPost(
        requestBody: DatasetExportRequest,
    ): CancelablePromise<DatasetExportResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/datasets/export',
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Export Dataset For Episode
     * @param episodeId
     * @returns DatasetExportResponse Successful Response
     * @throws ApiError
     */
    public static exportDatasetForEpisodeDatasetsEpisodeIdExportPost(
        episodeId: string,
    ): CancelablePromise<DatasetExportResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/datasets/{episode_id}/export',
            path: {
                'episode_id': episodeId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Dataset Export
     * @param exportId
     * @returns DatasetExportResponse Successful Response
     * @throws ApiError
     */
    public static getDatasetExportDatasetsExportsExportIdGet(
        exportId: string,
    ): CancelablePromise<DatasetExportResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/datasets/exports/{export_id}',
            path: {
                'export_id': exportId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Dataset Export By Episode
     * @param episodeId
     * @returns DatasetExportResponse Successful Response
     * @throws ApiError
     */
    public static getDatasetExportByEpisodeDatasetsEpisodesEpisodeIdGet(
        episodeId: string,
    ): CancelablePromise<DatasetExportResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/datasets/episodes/{episode_id}',
            path: {
                'episode_id': episodeId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
