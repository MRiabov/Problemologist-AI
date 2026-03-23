/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AssetType } from './AssetType';
/**
 * Reference to a persisted artifact included in a dataset row archive.
 */
export type DatasetRowArtifactReference = {
    path: string;
    family: string;
    asset_type?: (AssetType | null);
    source_surface: string;
    sha256: string;
    size_bytes?: (number | null);
};

