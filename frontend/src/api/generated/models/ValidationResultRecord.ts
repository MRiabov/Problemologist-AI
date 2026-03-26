/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { MultiRunResult } from './MultiRunResult';
/**
 * Persisted validation gate result for submission handoff checks.
 */
export type ValidationResultRecord = {
    success: boolean;
    message?: (string | null);
    timestamp: number;
    script_path?: (string | null);
    script_sha256?: (string | null);
    verification_result?: (MultiRunResult | null);
};

