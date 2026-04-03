/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { RendererCapabilities } from './RendererCapabilities';
import type { RenderMode } from './RenderMode';
import type { SimulatorBackendType } from './SimulatorBackendType';
export type SimulationRenderProvenance = {
    artifact_mode?: RenderMode;
    backend_type: SimulatorBackendType;
    backend_name: string;
    renderer_capabilities?: (RendererCapabilities | null);
    available_camera_names?: Array<string>;
    camera_candidates?: Array<string>;
    resolved_camera_name?: (string | null);
    used_default_view?: boolean;
    resolved_default_view_label?: (string | null);
    capture_interval_seconds?: (number | null);
    capture_interval_steps?: (number | null);
    captured_frame_count?: number;
    render_error?: (string | null);
};

