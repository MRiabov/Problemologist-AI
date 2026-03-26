/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AgentName } from './AgentName';
import type { EntryFailureDisposition } from './EntryFailureDisposition';
import type { EntryValidationError } from './EntryValidationError';
/**
 * Structured node-entry validation context persisted in episode metadata.
 */
export type EntryValidationContext = {
    node?: (AgentName | null);
    disposition?: (EntryFailureDisposition | null);
    reason_code?: (string | null);
    reroute_target?: (AgentName | null);
    errors?: Array<EntryValidationError>;
};

