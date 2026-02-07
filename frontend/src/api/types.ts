export interface Trace {
    id: number;
    episode_id: string;
    langfuse_trace_id?: string;
    raw_trace?: any;
    created_at: string;
}

export interface Asset {
    id: number;
    episode_id: string;
    asset_type: 'video' | 'mjcf' | 'image' | 'step' | 'stl';
    s3_path: string;
    created_at: string;
}

export interface Episode {
    id: string;
    task: string;
    status: 'running' | 'completed' | 'failed' | 'cancelled' | 'success';
    created_at: string;
    updated_at: string;
    skill_git_hash?: string;
    template_versions?: Record<string, string>;
    metadata_vars?: Record<string, any>;
    traces?: Trace[];
    assets?: Asset[];
}

export interface Skill {
    name: string;
    description: string;
}
