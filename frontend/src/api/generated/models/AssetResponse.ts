/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AssetType } from './AssetType';
export type AssetResponse = {
    id: number;
    user_session_id?: (string | null);
    asset_type: AssetType;
    s3_path: string;
    content?: (string | null);
    created_at: string;
};

