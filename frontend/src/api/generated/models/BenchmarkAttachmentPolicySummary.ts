/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BenchmarkPartAttachmentPolicy } from './BenchmarkPartAttachmentPolicy';
/**
 * Structured summary of a benchmark-owned fixture attachment policy.
 */
export type BenchmarkAttachmentPolicySummary = {
    part_id: string;
    label: string;
    allows_engineer_interaction: boolean;
    attachment_policy?: (BenchmarkPartAttachmentPolicy | null);
};

