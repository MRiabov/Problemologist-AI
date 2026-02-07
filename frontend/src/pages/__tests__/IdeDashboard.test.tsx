import { render, screen } from '@testing-library/react';
import { describe, it, expect } from 'vitest';
import IdeDashboard from '../IdeDashboard';
import { MemoryRouter } from 'react-router-dom';

describe('IdeDashboard', () => {
  it('renders the IDE layout with sidebar and main area', () => {
    render(
      <MemoryRouter>
        <IdeDashboard />
      </MemoryRouter>
    );

    expect(screen.getByText((content) => content.includes('CAD-Agent IDE'))).toBeInTheDocument();
    expect(screen.getByText((content) => content.includes('Episode History'))).toBeInTheDocument();
    expect(screen.getByText((content) => content.includes('generator.py'))).toBeInTheDocument();
    expect(screen.getByText((content) => content.includes('Agent Cognition'))).toBeInTheDocument();
    expect(screen.getByText((content) => content.includes('Log'))).toBeInTheDocument();
  });

  it('renders the metrics overlay', () => {
    render(
      <MemoryRouter>
        <IdeDashboard />
      </MemoryRouter>
    );

    expect(screen.getByText((content) => content.includes('Metrics'))).toBeInTheDocument();
    expect(screen.getByText(/420kJ/i)).toBeInTheDocument();
    expect(screen.getByText(/\$0.04/i)).toBeInTheDocument();
  });
});
