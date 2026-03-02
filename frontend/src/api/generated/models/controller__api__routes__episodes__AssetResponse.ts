/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AssetType } from './AssetType';
export type controller__api__routes__episodes__AssetResponse = {
    id: number;
    asset_type: AssetType;
    s3_path: string;
    content?: (string | null);
    created_at: string;
};

