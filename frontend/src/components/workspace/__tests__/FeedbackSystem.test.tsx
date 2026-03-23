import { render, screen, fireEvent } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { FeedbackSystem } from '../FeedbackSystem';
import * as apiClient from '../../../api/client';

vi.mock('../../../api/client', () => ({
  submitTraceFeedback: vi.fn(),
}));

describe('FeedbackSystem', () => {
    beforeEach(() => {
        vi.clearAllMocks();
        vi.useFakeTimers();
    });

    afterEach(() => {
        vi.useRealTimers();
    });

    it('renders correctly and submits feedback', async () => {
        (apiClient.submitTraceFeedback as any).mockResolvedValue({ status: 'success' });
        const onClose = vi.fn();
        render(
            <FeedbackSystem
                episodeId="test-episode"
                traceId={1}
                initialScore={1}
                onClose={onClose}
            />
        );

        expect(screen.getByText(/Agent Feedback/i)).toBeInTheDocument();

        const textarea = screen.getByPlaceholderText(/What was satisfying/);
        fireEvent.change(textarea, { target: { value: 'Great job!' } });

        const submitButton = screen.getByText(/Send Feedback/i);
        fireEvent.click(submitButton);

        await vi.runAllTimersAsync();
        expect(screen.getByText('Feedback Received')).toBeInTheDocument();
        expect(onClose).toHaveBeenCalled();
    });

    it('calls onClose when clicking cancel', () => {
        const onClose = vi.fn();
        render(
            <FeedbackSystem
                episodeId="test-episode"
                traceId={1}
                initialScore={0}
                onClose={onClose}
            />
        );

        const cancelButton = screen.getByText('Cancel');
        fireEvent.click(cancelButton);

        expect(onClose).toHaveBeenCalled();
    });
});
