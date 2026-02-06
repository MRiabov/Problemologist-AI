export interface Episode {
    id: string;
    task: string;
    status: 'running' | 'completed' | 'failed' | 'cancelled' | 'success';
    created_at: string;
}

export interface Skill {
    name: string;
    description: string;
}
