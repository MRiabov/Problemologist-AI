import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import BenchmarkGeneration from '../BenchmarkGeneration';
import * as apiClient from '../../api/client';
import { MemoryRouter } from 'react-router-dom';

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

  it('renders benchmark episodes on load', async () => {
    render(
      <MemoryRouter>
        <BenchmarkGeneration />
      </MemoryRouter>
    );

    await waitFor(() => {
      expect(screen.getByText('Sim Task 1')).toBeInTheDocument();
    });
  });

  it('triggers simulation run', async () => {
    (apiClient.runSimulation as any).mockResolvedValue({ status: 'accepted' });
    
    render(
      <MemoryRouter>
        <BenchmarkGeneration />
      </MemoryRouter>
    );

    const runButton = screen.getByText('RUN BENCHMARK');
    fireEvent.click(runButton);

    await waitFor(() => {
      expect(apiClient.runSimulation).toHaveBeenCalled();
      expect(screen.getByText('SIMULATING...')).toBeInTheDocument();
    });
  });

  it('shows MJCF placeholder when no asset is selected', async () => {
     render(
      <MemoryRouter>
        <BenchmarkGeneration />
      </MemoryRouter>
    );
    
    await waitFor(() => {
      expect(screen.getByText(/Procedural Benchmark Floor/)).toBeInTheDocument();
    });
  });

  it('selects an episode and updates the view', async () => {
    const fullEpisode = { 
        ...mockEpisodes[0], 
        assets: [{ asset_type: 'mjcf', s3_path: 'test.xml' }] 
    };
    (apiClient.fetchEpisode as any).mockResolvedValue(fullEpisode);

    render(
      <MemoryRouter>
        <BenchmarkGeneration />
      </MemoryRouter>
    );

    await waitFor(() => screen.getByText('Sim Task 1'));
    fireEvent.click(screen.getByText('Sim Task 1'));

    await waitFor(() => {
      expect(apiClient.fetchEpisode).toHaveBeenCalledWith('sim-1');
      expect(screen.getByText(/Load MJCF from: test.xml/)).toBeInTheDocument();
    });
  });
});
