import type { Episode, Skill } from './types';

const API_BASE = '/api';

export async function fetchEpisodes(): Promise<Episode[]> {
    const res = await fetch(`${API_BASE}/episodes/`);
    if (!res.ok) throw new Error('Failed to fetch episodes');
    return res.json();
}

export async function fetchSkills(): Promise<Skill[]> {
    const res = await fetch(`${API_BASE}/skills/`);
    if (!res.ok) throw new Error('Failed to fetch skills');
    return res.json();
}
