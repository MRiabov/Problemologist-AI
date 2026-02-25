/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BackupTriggerResponse } from '../models/BackupTriggerResponse';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class OperationsService {
    /**
     * Trigger Backup
     * Trigger the automated backup workflow.
     * @param xBackupSecret
     * @returns BackupTriggerResponse Successful Response
     * @throws ApiError
     */
    public static triggerBackupOpsBackupPost(
        xBackupSecret?: string,
    ): CancelablePromise<BackupTriggerResponse> {
        return __request(OpenAPI, {
            method: 'POST',
            url: '/ops/backup',
            headers: {
                'x-backup-secret': xBackupSecret,
            },
            errors: {
                422: `Validation Error`,
            },
        });
    }
}
