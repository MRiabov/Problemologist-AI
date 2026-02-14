import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { describe, it, expect, vi } from 'vitest';
import { FeedbackSystem } from '../FeedbackSystem';

describe('FeedbackSystem', () => {
  it('renders correctly and submits feedback', async () => {
    const onClose = vi.fn();
    render(<FeedbackSystem episodeId="test-episode" onClose={onClose} />);

    expect(screen.getByText('Feedback')).toBeInTheDocument();
    
    const textarea = screen.getByPlaceholderText(/What was satisfying/);
    fireEvent.change(textarea, { target: { value: 'Great job!' } });

    const submitButton = screen.getByText('Submit');
    fireEvent.click(submitButton);

    expect(screen.getByText('Feedback Received')).toBeInTheDocument();
    
    // Should call onClose after delay
    await waitFor(() => expect(onClose).toHaveBeenCalled(), { timeout: 3000 });
  });

  it('calls onClose when clicking cancel', () => {
    const onClose = vi.fn();
    render(<FeedbackSystem episodeId="test-episode" onClose={onClose} />);

    const cancelButton = screen.getByText('Cancel');
    fireEvent.click(cancelButton);

    expect(onClose).toHaveBeenCalled();
  });
});
