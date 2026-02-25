import { render, screen, fireEvent } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import ChatWindow from '../ChatWindow';
import { useEpisodes } from '../../../context/EpisodeContext';
import { useTheme } from '../../../context/ThemeContext';
import { MemoryRouter } from 'react-router-dom';
import { EpisodeStatus } from '../../../api/generated/models/EpisodeStatus';
import { TraceType } from '../../../api/generated/models/TraceType';

// Mock specific hooks to control state
vi.mock('../../../context/EpisodeContext', async () => {
    const actual = await vi.importActual('../../../context/EpisodeContext');
    return {
        ...actual,
        useEpisodes: vi.fn(),
    };
});

vi.mock('../../../context/ThemeContext', async () => {
    const actual = await vi.importActual('../../../context/ThemeContext');
    return {
        ...actual,
        useTheme: vi.fn(),
    };
});

// Mock SyntaxHighlighter to avoid heavy rendering
vi.mock('react-syntax-highlighter', () => ({
    Prism: ({ children }: any) => <pre>{children}</pre>,
}));

// Mock API client
vi.mock('../../api/client', () => ({
    runSimulation: vi.fn(),
}));

describe('ChatWindow', () => {
    const mockInterruptAgent = vi.fn();
    const mockConfirmBenchmark = vi.fn();
    const mockStartAgent = vi.fn();

    const defaultEpisodeContext = {
        isCreationMode: false,
        startAgent: mockStartAgent,
        continueAgent: vi.fn(),
        confirmBenchmark: mockConfirmBenchmark,
        interruptAgent: mockInterruptAgent,
        selectedEpisode: null,
        updateObjectives: vi.fn(),
        episodes: [],
        selectedContext: [],
        clearContext: vi.fn(),
    };

    beforeEach(() => {
        vi.clearAllMocks();
        (useTheme as any).mockReturnValue({ theme: 'dark' });
        (useEpisodes as any).mockReturnValue(defaultEpisodeContext);
    });

    it('renders interruption button when running', () => {
        const runningEpisode = { id: 'ep-1', status: EpisodeStatus.RUNNING };
        (useEpisodes as any).mockReturnValue({
            ...defaultEpisodeContext,
            isRunning: true,
            selectedEpisode: runningEpisode,
        });

        render(
            <MemoryRouter>
                <ChatWindow isRunning={true} traces={[]} />
            </MemoryRouter>
        );

        const stopButton = screen.getByRole('button', { name: /Stop Agent/i });
        
        fireEvent.click(stopButton);
        expect(mockInterruptAgent).toHaveBeenCalledWith('ep-1');
    });

    it('renders execution plan and confirm button when planned', async () => {
        const plannedEpisode = { 
            id: 'ep-1', 
            status: EpisodeStatus.RUNNING,
            metadata_vars: { detailed_status: 'PLANNED' } 
        };
        (useEpisodes as any).mockReturnValue({
            ...defaultEpisodeContext,
            selectedEpisode: plannedEpisode,
        });

        render(
            <MemoryRouter>
                <ChatWindow isRunning={false} traces={[]} />
            </MemoryRouter>
        );

        expect(screen.getByText(/Execution Plan Ready/i)).toBeInTheDocument();
        const confirmButton = screen.getByText(/Confirm & Start/i);
        fireEvent.click(confirmButton);
        
        expect(mockConfirmBenchmark).toHaveBeenCalledWith('ep-1', '');
    });

    it('renders traces correctly', () => {
        const traces = [
            { id: '1', trace_type: 'thought', content: 'I am thinking' },
            { id: '2', trace_type: TraceType.TOOL_START, name: 'view_file', content: '{"path": "test.py"}' },
        ];

        render(
            <MemoryRouter>
                <ChatWindow traces={traces as any} />
            </MemoryRouter>
        );

        expect(screen.getByText(/Thought for/i)).toBeInTheDocument();
        expect(screen.getByText(/Read/i)).toBeInTheDocument();
        expect(screen.getByText(/test.py/i)).toBeInTheDocument();
    });

    it('renders failure message when episode failed', () => {
        const failedEpisode = { 
            id: 'ep-1', 
            status: EpisodeStatus.FAILED,
            validation_logs: ["Error: failed to build"] 
        };
        (useEpisodes as any).mockReturnValue({
            ...defaultEpisodeContext,
            selectedEpisode: failedEpisode,
        });

        render(
            <MemoryRouter>
                <ChatWindow isRunning={false} />
            </MemoryRouter>
        );

        expect(screen.getByText(/Terminal failure/i)).toBeInTheDocument();
        expect(screen.getByText(/Error: failed to build/i)).toBeInTheDocument();
    });
});
