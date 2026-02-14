import { render, screen, fireEvent, waitFor } from '../../test/test-utils';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import BenchmarkGeneration from '../BenchmarkGeneration';
import * as apiClient from '../../api/client';

vi.mock('../../api/client', () => ({
  fetchEpisodes: vi.fn(),
  fetchEpisode: vi.fn(),
  runSimulation: vi.fn(),
  runAgent: vi.fn(),
  checkConnection: vi.fn().mockResolvedValue({ connected: true, isMockMode: false }),
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
    render(<BenchmarkGeneration />);

    await waitFor(() => {
      // Pipeline header
      expect(screen.getByText(/Benchmark Pipeline/i)).toBeInTheDocument();
      // Look for the "Simulation Preview" text
      expect(screen.getByText(/Simulation Preview/i)).toBeInTheDocument();
    });
  });

  it('triggers simulation run', async () => {
    (apiClient.runAgent as any).mockResolvedValue({ episode_id: 'new-ep' });
    
    render(<BenchmarkGeneration />);

    const input = screen.getByPlaceholderText(/Ask anything/);
    fireEvent.change(input, { target: { value: 'Start simulation' } });
    
    const sendButton = screen.getByLabelText('Send Message');
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(apiClient.runAgent).toHaveBeenCalled();
    });
  });
});
