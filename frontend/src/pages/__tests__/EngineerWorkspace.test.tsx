import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import EngineerWorkspace from '../EngineerWorkspace';
import * as apiClient from '../../api/client';
import { MemoryRouter } from 'react-router-dom';

vi.mock('../../api/client', () => ({
  fetchEpisodes: vi.fn(),
  fetchSkills: vi.fn(),
  fetchEpisode: vi.fn(),
  runAgent: vi.fn(),
}));

describe('EngineerWorkspace', () => {
  const mockEpisodes = [
    { id: '1', task: 'Task 1', status: 'completed', created_at: new Date().toISOString() },
    { id: '2', task: 'Task 2', status: 'running', created_at: new Date().toISOString() },
  ];
  const mockSkills = [{ name: 'Skill 1', description: 'Desc 1' }];

  beforeEach(() => {
    vi.clearAllMocks();
    (apiClient.fetchEpisodes as any).mockResolvedValue(mockEpisodes);
    (apiClient.fetchSkills as any).mockResolvedValue(mockSkills);
  });

  it('renders episodes and skills on load', async () => {
    render(
      <MemoryRouter>
        <EngineerWorkspace />
      </MemoryRouter>
    );

    await waitFor(() => {
      expect(screen.getByText(/Task 1/)).toBeInTheDocument();
      expect(screen.getByText(/Task 2/)).toBeInTheDocument();
      expect(screen.getByText('Skill 1')).toBeInTheDocument();
    });
  });

  it('handles episode selection', async () => {
    const fullEpisode = { ...mockEpisodes[0], traces: [], assets: [] };
    (apiClient.fetchEpisode as any).mockResolvedValue(fullEpisode);

    render(
      <MemoryRouter>
        <EngineerWorkspace />
      </MemoryRouter>
    );

    await waitFor(() => screen.getByText(/Task 1/));
    
    fireEvent.click(screen.getByText(/Task 1/));

    await waitFor(() => {
      expect(apiClient.fetchEpisode).toHaveBeenCalledWith('1');
    });
  });

  it('runs new agent task', async () => {
    (apiClient.runAgent as any).mockResolvedValue({ status: 'ok' });
    
    render(
      <MemoryRouter>
        <EngineerWorkspace />
      </MemoryRouter>
    );

    const input = screen.getByPlaceholderText(/Describe the task/);
    fireEvent.change(input, { target: { value: 'New Test Task' } });
    
    const solveButton = screen.getByText('SOLVE');
    fireEvent.click(solveButton);

    await waitFor(() => {
      expect(apiClient.runAgent).toHaveBeenCalledWith('New Test Task', expect.any(String));
    });
  });
});
