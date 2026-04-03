/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BaseEvent } from './BaseEvent';
import type { PreviewRenderingType } from './PreviewRenderingType';
import type { PreviewViewSpec } from './PreviewViewSpec';
/**
 * Response from preview design endpoint.
 */
export type PreviewDesignResponse = {
    success: boolean;
    status_text: string;
    message?: (string | null);
    job_id?: (string | null);
    queued?: boolean;
    view_count?: (number | null);
    view_specs?: Array<PreviewViewSpec>;
    artifact_path?: (string | null);
    manifest_path?: (string | null);
    rendering_type?: PreviewRenderingType;
    drafting?: boolean;
    pitch?: (number | null);
    yaw?: (number | null);
    image_path?: (string | null);
    /**
     * Base64-encoded preview image payload used internally by worker delegation layers.
     */
    image_bytes_base64?: (string | null);
    render_blobs_base64?: Record<string, string>;
    object_store_keys?: Record<string, string>;
    render_manifest_json?: (string | null);
    events?: Array<BaseEvent>;
};

