/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { RenderMode } from './RenderMode';
export type RendererCapabilities = {
    backend_name: string;
    artifact_modes_supported?: Array<RenderMode>;
    supports_default_view?: boolean;
    supports_named_cameras?: boolean;
    supports_rgb?: boolean;
    supports_depth?: boolean;
    supports_segmentation?: boolean;
    default_view_label?: (string | null);
};

