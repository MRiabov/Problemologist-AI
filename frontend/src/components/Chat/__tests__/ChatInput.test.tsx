import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { ChatInput } from '../ChatInput';
import type { TopologyNode } from '../../visualization/ModelBrowser';
import userEvent from '@testing-library/user-event';

// Mock Lucide icons
vi.mock('lucide-react', () => ({
    Zap: () => <div data-testid="icon-zap" />,
    Rocket: () => <div data-testid="icon-rocket" />,
    Plus: () => <div data-testid="icon-plus" />,
    Mic: () => <div data-testid="icon-mic" />,
    ArrowRight: () => <div data-testid="icon-arrow-right" />,
    Square: () => <div data-testid="icon-square" />,
    ChevronUp: () => <div data-testid="icon-chevron-up" />,
    ChevronDown: () => <div data-testid="icon-chevron-down" />
}));

describe('ChatInput', () => {
    const mockOnSendMessage = vi.fn();
    const mockOnInterrupt = vi.fn();
    const mockAddToContext = vi.fn();
    const mockSetShowObjectives = vi.fn();

    const defaultProps = {
        onSendMessage: mockOnSendMessage,
        isRunning: false,
        onInterrupt: mockOnInterrupt,
        selectedEpisode: { 
            id: 'ep-1', 
            assets: [
                { id: '1', s3_path: 'main.py', asset_type: 'file' },
                { id: '2', s3_path: 'utils.py', asset_type: 'file' }
            ] 
        } as any,
        selectedContext: [],
        topologyNodes: [
            { id: 'part-1', name: 'Bracket', type: 'part', children: [] }
        ] as TopologyNode[],
        addToContext: mockAddToContext,
        showObjectives: false,
        setShowObjectives: mockSetShowObjectives
    };

    beforeEach(() => {
        vi.clearAllMocks();
    });

    it('renders input field', () => {
        render(<ChatInput {...defaultProps} />);
        expect(screen.getByPlaceholderText(/Ask anything/i)).toBeInTheDocument();
    });

    it('handles text input', async () => {
        render(<ChatInput {...defaultProps} />);
        const input = screen.getByPlaceholderText(/Ask anything/i);
        await userEvent.type(input, 'Hello world');
        expect(input).toHaveValue('Hello world');
    });

    it('triggers suggestions when typing @', async () => {
        render(<ChatInput {...defaultProps} />);
        const input = screen.getByPlaceholderText(/Ask anything/i);
        
        // Type @ and some text to trigger search
        await userEvent.type(input, '@Brack');
        
        expect(screen.getByText('Bracket')).toBeInTheDocument();
        expect(screen.getByText('part')).toBeInTheDocument();
    });

    it('shows file suggestions', async () => {
        render(<ChatInput {...defaultProps} />);
        const input = screen.getByPlaceholderText(/Ask anything/i);
        
        await userEvent.type(input, '@main');
        
        expect(screen.getByText('main.py')).toBeInTheDocument();
        expect(screen.getByText('file')).toBeInTheDocument();
    });

    it('inserts suggestion on click', async () => {
        render(<ChatInput {...defaultProps} />);
        const input = screen.getByPlaceholderText(/Ask anything/i);
        
        await userEvent.type(input, 'Check out @Brack');
        // Wait for suggestion to appear
        const suggestion = await screen.findByText('Bracket');
        fireEvent.click(suggestion);

        expect(input).toHaveValue('Check out @Bracket');
        expect(mockAddToContext).toHaveBeenCalledWith(expect.objectContaining({
            type: 'cad',
            label: 'Bracket'
        }));
    });

    it('submits message and parses mentions/code refs', async () => {
        render(<ChatInput {...defaultProps} />);
        const input = screen.getByPlaceholderText(/Ask anything/i);
        
        await userEvent.type(input, 'Look at @main.py:10-20 and @part-1');
        
        const submitBtn = screen.getByLabelText('Send Message');
        fireEvent.click(submitBtn);

        expect(mockOnSendMessage).toHaveBeenCalledWith(
            'Look at @main.py:10-20 and @part-1',
            expect.objectContaining({
                mentions: expect.arrayContaining(['part-1']),
                code_references: expect.arrayContaining([
                    expect.objectContaining({
                        file_path: 'main.py',
                        start_line: 10,
                        end_line: 20
                    })
                ])
            })
        );
    });

    it('handles keyboard navigation for suggestions', async () => {
        render(<ChatInput {...defaultProps} />);
        const input = screen.getByPlaceholderText(/Ask anything/i);
        
        // Trigger multiple suggestions (assuming empty filter shows some or we type common char)
        await userEvent.type(input, '@'); 
        // Need to match something to show list. 'a' matches main.py and Bracket
        await userEvent.type(input, 'a');

        const suggestions = screen.getAllByRole('button');
        
        // Ensure suggestions are present
        expect(screen.getByText('Bracket')).toBeInTheDocument();
        expect(screen.getByText('main.py')).toBeInTheDocument();

        // Arrow down
        fireEvent.keyDown(input, { key: 'ArrowDown' });
        // Enter to select
        fireEvent.keyDown(input, { key: 'Enter' });

        // Verify value changed (it should not keep just @a)
        expect(input.value).not.toBe('@a');
    });

    it('calls onInterrupt when stop button clicked', () => {
        render(<ChatInput {...defaultProps} isRunning={true} />);
        
        const stopBtn = screen.getByLabelText('Stop Agent');
        fireEvent.click(stopBtn);
        
        expect(mockOnInterrupt).toHaveBeenCalled();
    });
});
