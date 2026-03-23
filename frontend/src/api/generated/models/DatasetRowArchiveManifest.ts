/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { DatasetRowArtifactReference } from './DatasetRowArtifactReference';
import type { DatasetRowBenchmarkAsset } from './DatasetRowBenchmarkAsset';
import type { DatasetRowLineage } from './DatasetRowLineage';
/**
 * Strict manifest persisted alongside a dataset-row archive.
 */
export type DatasetRowArchiveManifest = {
    export_id: string;
    created_at: string;
    lineage: DatasetRowLineage;
    source_benchmark_asset?: (DatasetRowBenchmarkAsset | null);
    artifact_references?: Array<DatasetRowArtifactReference>;
    artifact_families?: Array<string>;
    validation_notes?: Array<string>;
};

