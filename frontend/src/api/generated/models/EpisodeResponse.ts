/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AssetResponse } from './AssetResponse';
import type { EpisodeStatus } from './EpisodeStatus';
import type { TraceResponse } from './TraceResponse';
export type EpisodeResponse = {
    id: string;
    task: string;
    status: EpisodeStatus;
    created_at: string;
    updated_at: string;
    skill_git_hash?: (string | null);
    template_versions?: (Record<string, any> | null);
    metadata_vars?: (Record<string, any> | null);
    traces?: Array<TraceResponse>;
    assets?: Array<AssetResponse>;
};

