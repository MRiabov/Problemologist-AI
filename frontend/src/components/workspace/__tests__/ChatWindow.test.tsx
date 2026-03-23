import { render, screen, fireEvent } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import ChatWindow from '../ChatWindow';
import { useEpisodes } from '../../../context/EpisodeContext';
import { useTheme } from '../../../context/ThemeContext';
import { useUISettings } from '../../../context/UISettingsContext';
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

vi.mock('../../../context/UISettingsContext', async () => {
    const actual = await vi.importActual('../../../context/UISettingsContext');
    return {
        ...actual,
        useUISettings: vi.fn(),
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
        (useUISettings as any).mockReturnValue({ viewReasoning: false, setViewReasoning: vi.fn() });
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
            {
                id: '1',
                trace_type: TraceType.LLM_END,
                name: 'planner',
                content: 'I am thinking',
            },
            { id: '2', trace_type: TraceType.TOOL_START, name: 'view_file', content: '{"path": "test.py"}' },
        ];
        (useUISettings as any).mockReturnValue({ viewReasoning: true, setViewReasoning: vi.fn() });

        render(
            <MemoryRouter>
                <ChatWindow traces={traces as any} />
            </MemoryRouter>
        );

        expect(screen.getByText(/Reasoning after planner/i)).toBeInTheDocument();
        expect(screen.getByText(/Read/i)).toBeInTheDocument();
        expect(screen.getByText(/test.py/i)).toBeInTheDocument();
    });

    it('renders named LLM_END traces as reasoning when view reasoning is enabled', () => {
        const traces = [
            {
                id: '10',
                trace_type: TraceType.LLM_END,
                name: 'planner',
                content: 'First thought',
                metadata_vars: { reasoning_step_index: 0 },
            },
        ];
        (useUISettings as any).mockReturnValue({ viewReasoning: true, setViewReasoning: vi.fn() });

        render(
            <MemoryRouter>
                <ChatWindow traces={traces as any} />
            </MemoryRouter>
        );

        expect(screen.getByText(/Reasoning after planner · step 0/i)).toBeInTheDocument();
        fireEvent.click(screen.getByTestId('reasoning-span'));
        expect(screen.getByText(/First thought/i)).toBeInTheDocument();
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

    it('shows telemetry warning when reasoning is required but missing', () => {
        (useUISettings as any).mockReturnValue({ viewReasoning: false, setViewReasoning: vi.fn() });
        (useEpisodes as any).mockReturnValue({
            ...defaultEpisodeContext,
            selectedEpisode: {
                id: 'ep-telemetry',
                status: EpisodeStatus.COMPLETED,
                metadata_vars: { additional_info: { reasoning_required: true } },
            },
        });

        render(
            <MemoryRouter>
                <ChatWindow traces={[]} />
            </MemoryRouter>
        );

        expect(screen.getByTestId('reasoning-telemetry-warning')).toBeInTheDocument();
    });

    it('renders conversation length exceeded event in chat timeline', () => {
        const traces = [
            {
                id: '99',
                trace_type: TraceType.EVENT,
                name: 'conversation_length_exceeded',
                metadata_vars: {
                    threshold: 5000,
                    previous_length: 6200,
                    compacted_length: 2100,
                },
            },
        ];

        render(
            <MemoryRouter>
                <ChatWindow traces={traces as any} />
            </MemoryRouter>
        );

        expect(screen.getByTestId('conversation-length-exceeded')).toBeInTheDocument();
        expect(screen.getByText(/Conversation length limit exceeded/i)).toBeInTheDocument();
    });

    it('renders context usage indicator from episode metadata', () => {
        (useEpisodes as any).mockReturnValue({
            ...defaultEpisodeContext,
            selectedEpisode: {
                id: 'ep-ctx',
                status: EpisodeStatus.RUNNING,
                metadata_vars: {
                    additional_info: {
                        context_usage: {
                            used_tokens: 50000,
                            max_tokens: 225000,
                        },
                    },
                },
            },
        });

        render(
            <MemoryRouter>
                <ChatWindow traces={[]} />
            </MemoryRouter>
        );

        expect(screen.getByTestId('context-usage-indicator')).toBeInTheDocument();
        expect(screen.getByText(/Ctx 50,000 \/ 225,000 \(22.2%\)/i)).toBeInTheDocument();
    });
});
