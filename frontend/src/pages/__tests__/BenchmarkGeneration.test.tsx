import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import BenchmarkGeneration from '../BenchmarkGeneration';
import * as apiClient from '../../api/client';
import { MemoryRouter } from 'react-router-dom';
import { EpisodeProvider } from '../../context/EpisodeContext';

vi.mock('../../api/client', () => ({
  fetchEpisodes: vi.fn(),
  fetchEpisode: vi.fn(),
  runSimulation: vi.fn(),
}));

describe('BenchmarkGeneration', () => {
  const mockEpisodes = [
    { id: 'sim-1', task: 'Sim Task 1', status: 'completed', created_at: new Date().toISOString() },
  ];

  beforeEach(() => {
    vi.clearAllMocks();
    (apiClient.fetchEpisodes as any).mockResolvedValue(mockEpisodes);
  });

  it('renders benchmark pipeline correctly', async () => {
    render(
      <EpisodeProvider>
        <MemoryRouter>
          <BenchmarkGeneration />
        </MemoryRouter>
      </EpisodeProvider>
    );

    await waitFor(() => {
      // Pipeline header
      expect(screen.getByText(/Benchmark Pipeline/i)).toBeInTheDocument();
      // Look for the "Isolated" badge
      expect(screen.getByText(/Isolated/i)).toBeInTheDocument();
    });
  });

  it('triggers simulation run', async () => {
    (apiClient.runSimulation as any).mockResolvedValue({ status: 'accepted' });
    
    render(
      <EpisodeProvider>
        <MemoryRouter>
          <BenchmarkGeneration />
        </MemoryRouter>
      </EpisodeProvider>
    );

    const runButton = screen.getByRole('button', { name: /RUN PIPELINE/i });
    fireEvent.click(runButton);

    await waitFor(() => {
      expect(apiClient.runSimulation).toHaveBeenCalled();
    });
  });
});
