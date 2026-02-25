import { EpisodesService } from './generated/services/EpisodesService';
import { SkillsService } from './generated/services/SkillsService';
import { DefaultService } from './generated/services/DefaultService';
import { SimulationService } from './generated/services/SimulationService';
import type { controller__api__routes__episodes__EpisodeResponse as EpisodeResponse } from './generated/models/controller__api__routes__episodes__EpisodeResponse';
import type { Skill } from './generated/models/Skill';
import type { AgentRunResponse } from './generated/models/AgentRunResponse';
import type { BenchmarkGenerateResponse } from './generated/models/BenchmarkGenerateResponse';
import type { BenchmarkConfirmResponse } from './generated/models/BenchmarkConfirmResponse';
import type { BenchmarkObjectivesResponse } from './generated/models/BenchmarkObjectivesResponse';

export type { EpisodeResponse as Episode };
export type { Skill };

export async function fetchEpisodes(): Promise<EpisodeResponse[]> {
    return EpisodesService.listEpisodesEpisodesGet();
}

export async function fetchSkills(): Promise<Skill[]> {
    return SkillsService.listSkillsSkillsGet();
}

export async function fetchEpisode(id: string): Promise<EpisodeResponse> {
    return EpisodesService.getEpisodeEpisodesEpisodeIdGet(id);
}

export async function runAgent(task: string, sessionId: string, metadata?: Record<string, any>): Promise<AgentRunResponse> {
    return DefaultService.runAgentAgentRunPost({
        task,
        session_id: sessionId,
        metadata_vars: metadata
    });
}

export async function interruptEpisode(id: string): Promise<BenchmarkConfirmResponse> {
    return EpisodesService.interruptEpisodeEpisodesEpisodeIdInterruptPost(id);
}

export async function submitTraceFeedback(episodeId: string, traceId: number, score: number, comment?: string): Promise<any> {
    return EpisodesService.reportTraceFeedbackEpisodesEpisodeIdTracesTraceIdFeedbackPost(
        episodeId,
        traceId,
        { score, comment }
    );
}

export async function runSimulation(sessionId: string, compoundJson: string = '{}'): Promise<BenchmarkConfirmResponse> {
    return SimulationService.runSimulationSimulationRunPost({
        session_id: sessionId,
        compound_json: compoundJson
    });
}

export async function checkConnection(): Promise<{ connected: boolean; isMockMode: boolean }> {
    try {
        const health = await DefaultService.healthCheckHealthGet();
        return { 
            connected: true, 
            isMockMode: !!health.is_integration_test 
        };
    } catch (e) {
        console.error("Connection check failed:", e);
        return { 
            connected: false, 
            isMockMode: false 
        };
    }
}

// Manual API calls avoiding generator dependency
import { request as __request } from './generated/core/request';
import { OpenAPI } from './generated/core/OpenAPI';

export interface BenchmarkObjectives {
    max_cost?: number;
    max_weight?: number;
    target_quantity?: number;
}

export async function generateBenchmark(prompt: string, objectives?: BenchmarkObjectives): Promise<BenchmarkGenerateResponse> {
    return __request(OpenAPI, {
        method: 'POST',
        url: '/benchmark/generate',
        body: {
            prompt,
            max_cost: objectives?.max_cost,
            max_weight: objectives?.max_weight,
            target_quantity: objectives?.target_quantity
        },
        mediaType: 'application/json',
    });
}

export async function updateBenchmarkObjectives(sessionId: string, objectives: BenchmarkObjectives): Promise<BenchmarkObjectivesResponse> {
     return __request(OpenAPI, {
        method: 'POST',
        url: `/benchmark/${sessionId}/objectives`, // Updated URL path to match backend
        body: {
            max_cost: objectives.max_cost,
            max_weight: objectives.max_weight,
            target_quantity: objectives.target_quantity
        },
        mediaType: 'application/json',
    });
}

export async function confirmBenchmark(sessionId: string, comment?: string): Promise<BenchmarkConfirmResponse> {
    return __request(OpenAPI, {
        method: 'POST',
        url: `/benchmark/${sessionId}/confirm`,
        body: { comment },
        mediaType: 'application/json',
    });
}

export async function continueEpisode(id: string, message: string, metadata?: Record<string, any>): Promise<BenchmarkConfirmResponse> {
    return __request(OpenAPI, {
        method: 'POST',
        url: `/episodes/${id}/messages`,
        body: { 
            message,
            metadata_vars: metadata
        },
        mediaType: 'application/json',
    });
}

export async function rebuildModel(scriptPath: string): Promise<any> {
    return __request(OpenAPI, {
        method: 'POST',
        url: '/benchmark/build',
        body: {
            script_path: scriptPath
        },
        mediaType: 'application/json',
    });
}

export async function steerAgent(sessionId: string, text: string, metadata?: Record<string, any>): Promise<any> {
    return __request(OpenAPI, {
        method: 'POST',
        url: `/sessions/${sessionId}/steer`,
        body: {
            text,
            selections: metadata?.selections || [],
            code_references: metadata?.code_references || [],
            mentions: metadata?.mentions || []
        },
        mediaType: 'application/json',
    });
}
