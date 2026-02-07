/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { EpisodeResponse } from '../models/EpisodeResponse';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class EpisodesService {
    /**
     * List Episodes
     * List all agent episodes.
     * @returns EpisodeResponse Successful Response
     * @throws ApiError
     */
    public static listEpisodesEpisodesGet(): CancelablePromise<Array<EpisodeResponse>> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/episodes/',
        });
    }
    /**
     * Get Episode
     * Get a specific episode.
     * @param episodeId
     * @returns EpisodeResponse Successful Response
     * @throws ApiError
     */
    public static getEpisodeEpisodesEpisodeIdGet(
        episodeId: string,
    ): CancelablePromise<EpisodeResponse> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/episodes/{episode_id}',
            path: {
                'episode_id': episodeId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
