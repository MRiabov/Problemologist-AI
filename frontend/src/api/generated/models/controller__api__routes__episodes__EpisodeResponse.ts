/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { controller__api__routes__episodes__AssetResponse } from './controller__api__routes__episodes__AssetResponse';
import type { controller__api__routes__episodes__TraceResponse } from './controller__api__routes__episodes__TraceResponse';
import type { EpisodeStatus } from './EpisodeStatus';
export type controller__api__routes__episodes__EpisodeResponse = {
    id: string;
    user_session_id?: (string | null);
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
    traces?: (Array<controller__api__routes__episodes__TraceResponse> | null);
    assets?: (Array<controller__api__routes__episodes__AssetResponse> | null);
};

