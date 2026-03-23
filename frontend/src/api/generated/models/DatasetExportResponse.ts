/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { DatasetRowArchiveManifest } from './DatasetRowArchiveManifest';
import type { ObjectStoragePointer } from './ObjectStoragePointer';
import type { ResponseStatus } from './ResponseStatus';
export type DatasetExportResponse = {
    status: ResponseStatus;
    message: string;
    export_id: string;
    episode_id: string;
    archive: ObjectStoragePointer;
    manifest: DatasetRowArchiveManifest;
};

