import { EpisodesService } from './generated/services/EpisodesService';
import { SkillsService } from './generated/services/SkillsService';
import { DefaultService } from './generated/services/DefaultService';
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

export async function runSimulation(sessionId: string, compoundJson: string = '{}'): Promise<any> {
    return DefaultService.runSimulationSimulationRunPost({
        session_id: sessionId,
        compound_json: compoundJson
    });
}

export async function checkConnection(): Promise<boolean> {
    try {
        await DefaultService.healthHealthGet();
        return true;
    } catch (e) {
        return false;
    }
}
