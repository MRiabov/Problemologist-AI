/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AssetType } from './AssetType';
/**
 * A persisted artifact included in a replay bundle.
 */
export type ReplayArtifactRecord = {
    path: string;
    sha256: string;
    size_bytes: number;
    asset_type?: (AssetType | null);
};

