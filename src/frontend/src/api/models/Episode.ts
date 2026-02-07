/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
export type Episode = {
    id: string;
    task: string;
    status: Episode.status;
    created_at: string;
    updated_at?: string;
};
export namespace Episode {
    export enum status {
        RUNNING = 'running',
        COMPLETED = 'completed',
        FAILED = 'failed',
        CANCELLED = 'cancelled',
    }
}

