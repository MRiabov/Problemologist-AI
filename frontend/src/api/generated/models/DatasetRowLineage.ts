/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { EpisodeType } from './EpisodeType';
import type { GenerationKind } from './GenerationKind';
import type { SeedMatchMethod } from './SeedMatchMethod';
/**
 * Joinable lineage fields persisted with a dataset export row.
 */
export type DatasetRowLineage = {
    episode_id: string;
    benchmark_id: string;
    user_session_id?: (string | null);
    worker_session_id?: (string | null);
    episode_type: EpisodeType;
    seed_id?: (string | null);
    seed_dataset?: (string | null);
    seed_match_method?: (SeedMatchMethod | null);
    generation_kind?: (GenerationKind | null);
    parent_seed_id?: (string | null);
    is_integration_test?: (boolean | null);
    integration_test_id?: (string | null);
    simulation_run_id?: (string | null);
    cots_query_id?: (string | null);
    review_id?: (string | null);
    revision_hash: string;
    artifact_hash: string;
};

