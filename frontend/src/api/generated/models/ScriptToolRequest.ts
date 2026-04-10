/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { AgentName } from './AgentName';
import type { PreviewRenderingType } from './PreviewRenderingType';
import type { SimulatorBackendType } from './SimulatorBackendType';
export type ScriptToolRequest = {
    script_path?: string;
    agent_role?: AgentName;
    backend?: SimulatorBackendType;
    smoke_test_mode?: (boolean | null);
    bundle_base64?: (string | null);
    pitch?: (number | Array<number>);
    yaw?: (number | Array<number>);
    rgb?: (boolean | null);
    depth?: (boolean | null);
    segmentation?: (boolean | null);
    payload_path?: boolean;
    drafting?: boolean;
    rendering_type?: (PreviewRenderingType | null);
    reviewer_stage?: (AgentName | null);
    jitter_range?: (any[] | null);
    num_scenes?: (number | null);
    duration?: (number | null);
    seed?: (number | null);
    episode_id?: (string | null);
    stream_render_frames?: boolean;
};

