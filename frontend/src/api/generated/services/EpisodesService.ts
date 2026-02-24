/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { controller__api__routes__episodes__EpisodeResponse } from '../models/controller__api__routes__episodes__EpisodeResponse';
import type { FeedbackRequest } from '../models/FeedbackRequest';
import type { MessageRequest } from '../models/MessageRequest';
import type { ReviewRequest } from '../models/ReviewRequest';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class EpisodesService {
    /**
     * Get Episode Asset
     * Proxy asset requests to the worker.
     * @param episodeId
     * @param path
     * @returns any Successful Response
     * @throws ApiError
     */
    public static getEpisodeAssetEpisodesEpisodeIdAssetsPathGet(
        episodeId: string,
        path: string,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/episodes/{episode_id}/assets/{path}',
            path: {
                'episode_id': episodeId,
                'path': path,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Report Trace Feedback
     * Report feedback for a specific trace to Langfuse.
     * @param episodeId
     * @param traceId
     * @param requestBody
     * @returns any Successful Response
     * @throws ApiError
     */
    public static reportTraceFeedbackEpisodesEpisodeIdTracesTraceIdFeedbackPost(
        episodeId: string,
        traceId: number,
        requestBody: FeedbackRequest,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/episodes/{episode_id}/traces/{trace_id}/feedback',
            path: {
                'episode_id': episodeId,
                'trace_id': traceId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Review Episode
     * Submit a review for an episode.
     * @param episodeId
     * @param requestBody
     * @returns any Successful Response
     * @throws ApiError
     */
    public static reviewEpisodeEpisodesEpisodeIdReviewPost(
        episodeId: string,
        requestBody: ReviewRequest,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/episodes/{episode_id}/review',
            path: {
                'episode_id': episodeId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Continue Episode
     * Send a follow-up message to a running or completed episode.
     * @param episodeId
     * @param requestBody
     * @returns any Successful Response
     * @throws ApiError
     */
    public static continueEpisodeEpisodesEpisodeIdMessagesPost(
        episodeId: string,
        requestBody: MessageRequest,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/episodes/{episode_id}/messages',
            path: {
                'episode_id': episodeId,
            },
            body: requestBody,
            mediaType: 'application/json',
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Episode Schematic
     * Get the electronics schematic for an episode in a format compatible with tscircuit.
     * @param episodeId
     * @returns any Successful Response
     * @throws ApiError
     */
    public static getEpisodeSchematicEpisodesEpisodeIdElectronicsSchematicGet(
        episodeId: string,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/episodes/{episode_id}/electronics/schematic',
            path: {
                'episode_id': episodeId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * List Episodes
     * List all agent episodes.
     * @param limit
     * @param offset
     * @returns controller__api__routes__episodes__EpisodeResponse Successful Response
     * @throws ApiError
     */
    public static listEpisodesEpisodesGet(
        limit: number = 100,
        offset?: number,
    ): CancelablePromise<Array<controller__api__routes__episodes__EpisodeResponse>> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/episodes/',
            query: {
                'limit': limit,
                'offset': offset,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Get Episode
     * Get a specific episode.
     * @param episodeId
     * @returns controller__api__routes__episodes__EpisodeResponse Successful Response
     * @throws ApiError
     */
    public static getEpisodeEpisodesEpisodeIdGet(
        episodeId: string,
    ): CancelablePromise<controller__api__routes__episodes__EpisodeResponse> {
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
    /**
     * Delete Episode
     * Delete an episode and its associated data.
     * @param episodeId
     * @returns any Successful Response
     * @throws ApiError
     */
    public static deleteEpisodeEpisodesEpisodeIdDelete(
        episodeId: string,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'DELETE',
            url: '/episodes/{episode_id}',
            path: {
                'episode_id': episodeId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
    /**
     * Interrupt Episode
     * Interrupt a running episode.
     * @param episodeId
     * @returns any Successful Response
     * @throws ApiError
     */
    public static interruptEpisodeEpisodesEpisodeIdInterruptPost(
        episodeId: string,
    ): CancelablePromise<any> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/episodes/{episode_id}/interrupt',
            path: {
                'episode_id': episodeId,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
