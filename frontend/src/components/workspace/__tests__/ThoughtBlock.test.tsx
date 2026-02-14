import { render, screen, fireEvent } from '@testing-library/react';
import { describe, it, expect } from 'vitest';
import { ThoughtBlock } from '../ThoughtBlock';

describe('ThoughtBlock', () => {
  it('renders correctly with duration', () => {
    render(<ThoughtBlock duration={5} content="Thinking about logic" />);
    expect(screen.getByText('Thought for 5s')).toBeInTheDocument();
  });

  it('shows content only when expanded', () => {
    const content = "Thinking about logic";
    render(<ThoughtBlock duration={5} content={content} />);
    
    // Content should not be visible initially
    expect(screen.queryByText(content)).not.toBeInTheDocument();
    
    // Click to expand
    fireEvent.click(screen.getByRole('button'));
    
    // Content should be visible now
    expect(screen.getByText(content)).toBeInTheDocument();
  });

  it('returns null if content and duration are missing', () => {
    const { container } = render(<ThoughtBlock content="" duration={0} />);
    expect(container.firstChild).toBeNull();
  });
});
