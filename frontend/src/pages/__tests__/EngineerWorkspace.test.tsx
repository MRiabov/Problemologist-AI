import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import EngineerWorkspace from '../EngineerWorkspace';
import * as apiClient from '../../api/client';
import { MemoryRouter } from 'react-router-dom';
import { EpisodeProvider } from '../../context/EpisodeContext';

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

  beforeEach(() => {
    vi.clearAllMocks();
    (apiClient.fetchEpisodes as any).mockResolvedValue(mockEpisodes);
  });

  it('renders correctly with layout components', async () => {
    render(
      <EpisodeProvider>
        <MemoryRouter>
          <EngineerWorkspace />
        </MemoryRouter>
      </EpisodeProvider>
    );

    // Verify workspace-specific elements
    expect(screen.getByPlaceholderText(/Describe a mechanical task/i)).toBeInTheDocument();
    
    // Middle column components
    expect(screen.getByText(/Execution Traces/i)).toBeInTheDocument();
    expect(screen.getByText(/Critic Audit/i)).toBeInTheDocument();

    // Right column components (ArtifactView)
    expect(screen.getByText(/Explorer/i)).toBeInTheDocument();
  });

  it('runs new agent task', async () => {
    (apiClient.runAgent as any).mockResolvedValue({ status: 'ok', id: 'new-id', traces: [], assets: [] });
    
    render(
      <EpisodeProvider>
        <MemoryRouter>
          <EngineerWorkspace />
        </MemoryRouter>
      </EpisodeProvider>
    );

    const input = screen.getByPlaceholderText(/Describe a mechanical task/);
    fireEvent.change(input, { target: { value: 'New Test Task' } });
    
    const solveButton = screen.getByText('SOLVE');
    fireEvent.click(solveButton);

    await waitFor(() => {
      // Expect two arguments: task and session_id
      expect(apiClient.runAgent).toHaveBeenCalledWith('New Test Task', expect.any(String));
    });
  });
});
