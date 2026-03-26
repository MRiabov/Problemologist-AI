/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BenchmarkAttachmentMethod } from './BenchmarkAttachmentMethod';
import type { BenchmarkPartDrillPolicy } from './BenchmarkPartDrillPolicy';
/**
 * Benchmark-owned attachment policy for an environment part.
 */
export type BenchmarkPartAttachmentPolicy = {
    attachment_methods?: Array<BenchmarkAttachmentMethod>;
    drill_policy?: (BenchmarkPartDrillPolicy | null);
    notes?: (string | null);
};

