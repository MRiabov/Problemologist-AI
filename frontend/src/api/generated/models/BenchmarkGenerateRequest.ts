/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AgentName } from './AgentName';
import type { GenerationKind } from './GenerationKind';
import type { SimulatorBackendType } from './SimulatorBackendType';
export type BenchmarkGenerateRequest = {
    prompt: string;
    session_id?: (string | null);
    start_node?: (AgentName | null);
    max_cost?: (number | null);
    max_weight?: (number | null);
    target_quantity?: (number | null);
    seed_id?: (string | null);
    seed_dataset?: (string | null);
    generation_kind?: (GenerationKind | null);
    backend?: SimulatorBackendType;
};

