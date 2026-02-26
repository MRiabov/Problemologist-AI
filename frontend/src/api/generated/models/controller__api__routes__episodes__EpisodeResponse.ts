/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AssetResponse } from './AssetResponse';
import type { EpisodeStatus } from './EpisodeStatus';
import type { TraceResponse } from './TraceResponse';
export type controller__api__routes__episodes__EpisodeResponse = {
    id: string;
    task: string;
    status: EpisodeStatus;
    created_at: string;
    updated_at: string;
    skill_git_hash?: (string | null);
    template_versions?: (Record<string, any> | null);
    metadata_vars?: (Record<string, any> | null);
    todo_list?: (Record<string, any> | null);
    journal?: (string | null);
    plan?: (string | null);
    validation_logs?: (Array<string> | null);
    last_trace_id?: (number | null);
    traces?: (Array<TraceResponse> | null);
    assets?: (Array<AssetResponse> | null);
};

