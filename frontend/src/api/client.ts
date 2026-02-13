import { EpisodesService } from './generated/services/EpisodesService';
import { SkillsService } from './generated/services/SkillsService';
import { DefaultService } from './generated/services/DefaultService';
import { SimulationService } from './generated/services/SimulationService';
import type { EpisodeResponse } from './generated/models/EpisodeResponse';
import type { Skill } from './generated/models/Skill';

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

export async function runAgent(task: string, sessionId: string): Promise<any> {
    return DefaultService.runAgentAgentRunPost({
        task,
        session_id: sessionId
    });
}

export async function interruptEpisode(id: string): Promise<any> {
    return EpisodesService.interruptEpisodeEpisodesEpisodeIdInterruptPost(id);
}

export async function submitTraceFeedback(episodeId: string, traceId: number, score: number, comment?: string): Promise<any> {
    return EpisodesService.reportTraceFeedbackEpisodesEpisodeIdTracesTraceIdFeedbackPost(
        episodeId,
        traceId,
        { score, comment }
    );
}

export async function runSimulation(sessionId: string, compoundJson: string = '{}'): Promise<any> {
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

export async function generateBenchmark(prompt: string, objectives?: BenchmarkObjectives): Promise<any> {
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

export async function updateBenchmarkObjectives(sessionId: string, objectives: BenchmarkObjectives): Promise<any> {
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
