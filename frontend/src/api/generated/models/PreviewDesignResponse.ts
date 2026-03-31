/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { BaseEvent } from './BaseEvent';
import type { PreviewRenderingType } from './PreviewRenderingType';
/**
 * Response from preview design endpoint.
 */
export type PreviewDesignResponse = {
    success: boolean;
    status_text: string;
    message?: (string | null);
    artifact_path?: (string | null);
    manifest_path?: (string | null);
    rendering_type?: PreviewRenderingType;
    pitch?: (number | null);
    yaw?: (number | null);
    image_path?: (string | null);
    /**
     * Base64-encoded preview image payload used internally by worker delegation layers.
     */
    image_bytes_base64?: (string | null);
    render_manifest_json?: (string | null);
    events?: Array<BaseEvent>;
};

