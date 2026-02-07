import type { Episode, Skill } from './types';

const API_BASE = '/api';
const BASE_URL = typeof window !== 'undefined' ? window.location.origin : 'http://localhost:3000';

export async function fetchEpisodes(): Promise<Episode[]> {
    const res = await fetch(`${BASE_URL}${API_BASE}/episodes/`);
    if (!res.ok) throw new Error('Failed to fetch episodes');
    return res.json();
}

export async function fetchSkills(): Promise<Skill[]> {
    const res = await fetch(`${BASE_URL}${API_BASE}/skills/`);
    if (!res.ok) throw new Error('Failed to fetch skills');
    return res.json();
}

export async function fetchEpisode(id: string): Promise<Episode> {
    const res = await fetch(`${BASE_URL}${API_BASE}/episodes/${id}`);
    if (!res.ok) throw new Error('Failed to fetch episode');
    return res.json();
}

export async function runAgent(task: string, sessionId: string): Promise<any> {
    const res = await fetch(`${BASE_URL}${API_BASE}/agent/run`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ task, session_id: sessionId })
    });
    if (!res.ok) throw new Error('Failed to run agent');
    return res.json();
}

export async function runSimulation(sessionId: string, compoundJson: string = '{}'): Promise<any> {
    const res = await fetch(`${BASE_URL}${API_BASE}/simulation/run`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ session_id: sessionId, compound_json: compoundJson })
    });
    if (!res.ok) throw new Error('Failed to run simulation');
    return res.json();
}
