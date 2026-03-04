/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CustomObjectives } from './CustomObjectives';
/**
 * Structured metadata for episodes and benchmark sessions.
 */
export type EpisodeMetadata = {
    worker_session_id?: (string | null);
    benchmark_id?: (string | null);
    custom_objectives?: (CustomObjectives | null);
    detailed_status?: (string | null);
    error?: (string | null);
    variant_id?: (string | null);
    seed?: (number | null);
    prior_episode_id?: (string | null);
    is_optimality_check?: (boolean | null);
    fidelity_check?: (boolean | null);
    tolerance?: (number | null);
    is_reused?: (boolean | null);
    episode_type?: ('benchmark' | 'engineer' | null);
    validation_logs?: Array<string>;
    prompt?: (string | null);
    plan?: (Record<string, any> | null);
    cost?: (number | null);
    weight?: (number | null);
    seed_id?: (string | null);
    seed_dataset?: (string | null);
    seed_match_method?: (string | null);
    generation_kind?: (string | null);
    parent_seed_id?: (string | null);
    additional_info?: Record<string, any>;
};

