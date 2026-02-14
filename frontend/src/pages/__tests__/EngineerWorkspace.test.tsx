import { render, screen, fireEvent, waitFor } from '../../test/test-utils';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import EngineerWorkspace from '../EngineerWorkspace';
import * as apiClient from '../../api/client';

vi.mock('../../api/client', () => ({
  fetchEpisodes: vi.fn(),
  fetchSkills: vi.fn(),
  fetchEpisode: vi.fn(),
  runAgent: vi.fn(),
  checkConnection: vi.fn().mockResolvedValue({ connected: true, isMockMode: false }),
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
    render(<EngineerWorkspace />);

    // Verify workspace-specific elements
    expect(screen.getByPlaceholderText(/Ask anything/i)).toBeInTheDocument();
    
    // Middle column components (ChatWindow header)
    expect(screen.getAllByText(/Engineer Workspace/i).length).toBeGreaterThan(0);

    // Right column components (ArtifactView)
    expect(screen.getByText(/Explorer/i)).toBeInTheDocument();
  });

  it('runs new agent task', async () => {
    (apiClient.runAgent as any).mockResolvedValue({ episode_id: 'new-id' });
    
    render(<EngineerWorkspace />);

    const input = screen.getByPlaceholderText(/Ask anything/);
    fireEvent.change(input, { target: { value: 'New Test Task' } });
    
    const sendButton = screen.getByLabelText('Send Message');
    fireEvent.click(sendButton);

    await waitFor(() => {
      // Expect two arguments: task and session_id (and 3rd is undefined)
      expect(apiClient.runAgent).toHaveBeenCalledWith('New Test Task', expect.any(String), undefined);
    });
  });
});
