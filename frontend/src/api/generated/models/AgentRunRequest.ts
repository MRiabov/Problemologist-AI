/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
export type AgentRunRequest = {
    /**
     * The task for the agent to perform.
     */
    task: string;
    /**
     * Session ID for the worker.
     */
    session_id: string;
    /**
     * Additional metadata for the episode.
     */
    metadata_vars?: (Record<string, any> | null);
    /**
     * Git hash of the skills used for this run.
     */
    skill_git_hash?: (string | null);
    /**
     * The name of the agent to run.
     */
    agent_name?: string;
};

