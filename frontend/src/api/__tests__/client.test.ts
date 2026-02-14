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
      status: 200,
      headers: new Headers({ 'Content-Type': 'application/json' }),
      json: async () => mockEpisodes,
    });

    const result = await fetchEpisodes();
    expect(global.fetch).toHaveBeenCalledWith(expect.stringContaining('http://localhost:13000/episodes/'), expect.any(Object));
    expect(result).toEqual(mockEpisodes);
  });

  it('fetchSkills calls the correct endpoint', async () => {
    const mockSkills = [{ name: 'skill1', description: 'desc1' }];
    (global.fetch as any).mockResolvedValueOnce({
      ok: true,
      status: 200,
      headers: new Headers({ 'Content-Type': 'application/json' }),
      json: async () => mockSkills,
    });

    const result = await fetchSkills();
    expect(global.fetch).toHaveBeenCalledWith(expect.stringContaining('http://localhost:13000/skills/'), expect.any(Object));
    expect(result).toEqual(mockSkills);
  });

  it('fetchEpisode calls the correct endpoint', async () => {
    const mockEpisode = { id: '1', task: 'test', status: 'completed' };
    (global.fetch as any).mockResolvedValueOnce({
      ok: true,
      status: 200,
      headers: new Headers({ 'Content-Type': 'application/json' }),
      json: async () => mockEpisode,
    });

    const result = await fetchEpisode('1');
    expect(global.fetch).toHaveBeenCalledWith('http://localhost:13000/episodes/1', expect.any(Object));
    expect(result).toEqual(mockEpisode);
  });

  it('runAgent sends correct POST request', async () => {
    (global.fetch as any).mockResolvedValueOnce({
      ok: true,
      status: 200,
      headers: new Headers({ 'Content-Type': 'application/json' }),
      json: async () => ({ status: 'ok' }),
    });

    await runAgent('do something', 'session123');
    expect(global.fetch).toHaveBeenCalledWith('http://localhost:13000/agent/run', expect.objectContaining({
      method: 'POST',
      body: JSON.stringify({ task: 'do something', session_id: 'session123' }),
    }));
  });

  it('runSimulation sends correct POST request', async () => {
    (global.fetch as any).mockResolvedValueOnce({
      ok: true,
      status: 200,
      headers: new Headers({ 'Content-Type': 'application/json' }),
      json: async () => ({ status: 'ok' }),
    });

    await runSimulation('session123', '{"data": "test"}');
    expect(global.fetch).toHaveBeenCalledWith('http://localhost:13000/simulation/run', expect.objectContaining({
      method: 'POST',
      body: JSON.stringify({ session_id: 'session123', compound_json: '{"data": "test"}' }),
    }));
  });

  it('throws error when response is not ok', async () => {
    (global.fetch as any).mockResolvedValueOnce({
      ok: false,
      status: 500,
      statusText: 'Internal Server Error',
      headers: new Headers({ 'Content-Type': 'application/json' }),
      json: async () => ({ message: 'Error' }),
    });

    await expect(fetchEpisodes()).rejects.toThrow('Internal Server Error');
  });
});
