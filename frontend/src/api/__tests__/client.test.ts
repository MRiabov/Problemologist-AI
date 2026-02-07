import { describe, it, expect, vi, beforeEach } from 'vitest';
import { fetchEpisodes, fetchSkills, fetchEpisode, runAgent, runSimulation } from '../client';

describe('API Client', () => {
  beforeEach(() => {
    vi.stubGlobal('fetch', vi.fn());
  });

  it('fetchEpisodes calls the correct endpoint', async () => {
    const mockEpisodes = [{ id: '1', task: 'test', status: 'running', created_at: 'now' }];
    (global.fetch as any).mockResolvedValueOnce({
      ok: true,
      json: async () => mockEpisodes,
    });

    const result = await fetchEpisodes();
    expect(global.fetch).toHaveBeenCalledWith('http://localhost:3000/api/episodes/');
    expect(result).toEqual(mockEpisodes);
  });

  it('fetchSkills calls the correct endpoint', async () => {
    const mockSkills = [{ name: 'skill1', description: 'desc1' }];
    (global.fetch as any).mockResolvedValueOnce({
      ok: true,
      json: async () => mockSkills,
    });

    const result = await fetchSkills();
    expect(global.fetch).toHaveBeenCalledWith('http://localhost:3000/api/skills/');
    expect(result).toEqual(mockSkills);
  });

  it('fetchEpisode calls the correct endpoint', async () => {
    const mockEpisode = { id: '1', task: 'test', status: 'completed' };
    (global.fetch as any).mockResolvedValueOnce({
      ok: true,
      json: async () => mockEpisode,
    });

    const result = await fetchEpisode('1');
    expect(global.fetch).toHaveBeenCalledWith('http://localhost:3000/api/episodes/1');
    expect(result).toEqual(mockEpisode);
  });

  it('runAgent sends correct POST request', async () => {
    (global.fetch as any).mockResolvedValueOnce({
      ok: true,
      json: async () => ({ status: 'ok' }),
    });

    await runAgent('do something', 'session123');
    expect(global.fetch).toHaveBeenCalledWith('http://localhost:3000/api/agent/run', expect.objectContaining({
      method: 'POST',
      body: JSON.stringify({ task: 'do something', session_id: 'session123' }),
    }));
  });

  it('runSimulation sends correct POST request', async () => {
    (global.fetch as any).mockResolvedValueOnce({
      ok: true,
      json: async () => ({ status: 'ok' }),
    });

    await runSimulation('session123', '{"data": "test"}');
    expect(global.fetch).toHaveBeenCalledWith('http://localhost:3000/api/simulation/run', expect.objectContaining({
      method: 'POST',
      body: JSON.stringify({ session_id: 'session123', compound_json: '{"data": "test"}' }),
    }));
  });

  it('throws error when response is not ok', async () => {
    (global.fetch as any).mockResolvedValueOnce({
      ok: false,
    });

    await expect(fetchEpisodes()).rejects.toThrow('Failed to fetch episodes');
  });
});
