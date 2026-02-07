import { useQuery } from '@tanstack/react-query';
import { DefaultService } from '../../api';

export const RUNS_QUERY_KEY = ['runs'];

export function useRuns() {
  return useQuery({
    queryKey: RUNS_QUERY_KEY,
    queryFn: () => DefaultService.listEpisodes(),
  });
}

export function useRun(id: string) {
  return useQuery({
    queryKey: [...RUNS_QUERY_KEY, id],
    queryFn: () => DefaultService.getEpisode(id),
    enabled: !!id,
  });
}
