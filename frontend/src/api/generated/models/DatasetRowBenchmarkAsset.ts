/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * Snapshot of the benchmark asset surface used to materialize a dataset row.
 */
export type DatasetRowBenchmarkAsset = {
    benchmark_id: string;
    mjcf_url: string;
    build123d_url: string;
    preview_bundle_url: string;
    random_variants?: Array<string>;
    difficulty_score?: (number | null);
    benchmark_metadata?: Record<string, any>;
};

