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

    expect(screen.getByText(/CAD-Agent IDE/)).toBeInTheDocument();
    expect(screen.getByText(/Episode History/i)).toBeInTheDocument();
    expect(screen.getByText('generator.py')).toBeInTheDocument();
    expect(screen.getByText(/Agent Cognition Stream/i)).toBeInTheDocument();
    expect(screen.getByText(/Execution Log/i)).toBeInTheDocument();
  });

  it('renders the metrics overlay', () => {
    render(
      <MemoryRouter>
        <IdeDashboard />
      </MemoryRouter>
    );

    expect(screen.getByText(/Live Metrics/i)).toBeInTheDocument();
    expect(screen.getByText(/420kJ/i)).toBeInTheDocument();
    expect(screen.getByText(/\$0.04/i)).toBeInTheDocument();
  });
});
