/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * Runtime stop context for engineer-owned payload trajectory monitoring.
 */
export type PayloadTrajectoryMonitorState = {
    tracked_body_names?: Array<string>;
    last_checked_anchor_index?: (number | null);
    last_checked_anchor_t_s?: (number | null);
    monitor_sample_stride_s?: (number | null);
    configured_consecutive_miss_count?: (number | null);
    consecutive_miss_count?: number;
    observed_position_mm?: (any[] | null);
    observed_rotation_deg?: (any[] | null);
    position_error_mm?: (any[] | null);
    rotation_error_deg?: (any[] | null);
    required_first_contacts?: Array<string>;
    observed_first_contacts?: Array<string>;
    terminal_goal_zone_proven?: boolean;
    failure_detail?: (string | null);
};

