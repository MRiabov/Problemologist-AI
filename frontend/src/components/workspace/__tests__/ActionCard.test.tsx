import { render, screen } from '@testing-library/react';
import { describe, it, expect, vi } from 'vitest';
import { ActionCard } from '../ActionCard';
import { EpisodeProvider } from '../../../context/EpisodeContext';

// Mock getFileIconInfo to avoid complexity
vi.mock('../../../lib/fileIcons', () => ({
  getFileIconInfo: (fileName: string) => ({
    icon: () => <div data-testid="file-icon" />,
    color: '#000'
  })
}));

describe('ActionCard', () => {
  const mockTrace = (name: string, content: string = '{}') => ({
    id: 'trace-1',
    trace_type: 'tool_start',
    name,
    content,
    created_at: new Date().toISOString()
  });

  it('renders search tool correctly', () => {
    render(
      <EpisodeProvider>
        <ActionCard trace={mockTrace('codebase_search', JSON.stringify({ Query: 'test' }))} />
      </EpisodeProvider>
    );
    expect(screen.getByText('Searched')).toBeInTheDocument();
  });

  it('renders read tool and extracts fileName', () => {
    render(
      <EpisodeProvider>
        <ActionCard trace={mockTrace('view_file', JSON.stringify({ AbsolutePath: '/src/main.py' }))} />
      </EpisodeProvider>
    );
    expect(screen.getByText('Read')).toBeInTheDocument();
    expect(screen.getByText('main.py')).toBeInTheDocument();
  });

  it('renders edit tool correctly', () => {
    render(
      <EpisodeProvider>
        <ActionCard trace={mockTrace('write_to_file', JSON.stringify({ TargetFile: '/src/utils.py' }))} />
      </EpisodeProvider>
    );
    expect(screen.getByText('Edited')).toBeInTheDocument();
    expect(screen.getByText('utils.py')).toBeInTheDocument();
  });

  it('handles non-JSON content for path extraction', () => {
    render(
      <EpisodeProvider>
        <ActionCard trace={mockTrace('view_file', 'AbsolutePath: /src/config.yaml')} />
      </EpisodeProvider>
    );
    expect(screen.getByText('config.yaml')).toBeInTheDocument();
  });
});
