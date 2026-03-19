/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AgentName } from './AgentName';
import type { SimulatorBackendType } from './SimulatorBackendType';
export type ScriptToolRequest = {
    script_path?: string;
    agent_role?: AgentName;
    backend?: SimulatorBackendType;
    smoke_test_mode?: (boolean | null);
    reviewer_stage?: ('benchmark_reviewer' | 'engineering_execution_reviewer' | 'electronics_reviewer' | null);
};

