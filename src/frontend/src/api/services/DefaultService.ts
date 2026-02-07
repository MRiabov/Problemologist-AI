/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { Episode } from '../models/Episode';
import type { Skill } from '../models/Skill';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class DefaultService {
    /**
     * List Episodes
     * @returns Episode Successful Response
     * @throws ApiError
     */
    public static listEpisodes(): CancelablePromise<Array<Episode>> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/episodes/',
        });
    }
    /**
     * Get Episode
     * @param episodeId
     * @returns Episode Successful Response
     * @throws ApiError
     */
    public static getEpisode(
        episodeId: string,
    ): CancelablePromise<Episode> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/episodes/{episode_id}',
            path: {
                'episode_id': episodeId,
            },
        });
    }
    /**
     * List Skills
     * @returns Skill Successful Response
     * @throws ApiError
     */
    public static listSkills(): CancelablePromise<Array<Skill>> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/skills/',
        });
    }
}
