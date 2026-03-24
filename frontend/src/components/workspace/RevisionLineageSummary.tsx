import { useMemo } from "react";
import type { Episode } from "../../api/client";
import { EpisodeStatus } from "../../api/generated/models/EpisodeStatus";
import { EpisodeType } from "../../api/generated/models/EpisodeType";
import { cn } from "../../lib/utils";
import { Badge } from "../ui/badge";

interface RevisionLineageSummaryProps {
  episodes: Episode[];
  selectedEpisode: Episode | null;
  onSelectEpisode: (episodeId: string) => void | Promise<void>;
}

function shortId(value: string | null | undefined): string {
  if (!value) return "n/a";
  return value.length > 8 ? value.slice(0, 8) : value;
}

export function getEngineerRevisionLineage(
  episodes: Episode[],
  selectedEpisode: Episode | null,
): Episode[] {
  if (
    !selectedEpisode ||
    selectedEpisode.metadata_vars?.episode_type !== EpisodeType.ENGINEER
  ) {
    return [];
  }

  const benchmarkId = selectedEpisode.metadata_vars?.benchmark_id?.trim();
  if (!benchmarkId) {
    return [];
  }

  return episodes
    .filter(
      (episode) =>
        episode.metadata_vars?.episode_type === EpisodeType.ENGINEER &&
        episode.metadata_vars?.benchmark_id?.trim() === benchmarkId,
    )
    .slice()
    .sort((a, b) => {
      const aTime = new Date(a.created_at).getTime();
      const bTime = new Date(b.created_at).getTime();
      if (aTime !== bTime) {
        return aTime - bTime;
      }
      return String(a.id).localeCompare(String(b.id));
    });
}

export default function RevisionLineageSummary({
  episodes,
  selectedEpisode,
  onSelectEpisode,
}: RevisionLineageSummaryProps) {
  const revisionEpisodes = useMemo(
    () => getEngineerRevisionLineage(episodes, selectedEpisode),
    [
      episodes,
      selectedEpisode?.id,
      selectedEpisode?.metadata_vars?.benchmark_id,
      selectedEpisode?.metadata_vars?.episode_type,
    ],
  );

  if (!selectedEpisode || revisionEpisodes.length === 0) {
    return null;
  }

  const benchmarkId = selectedEpisode.metadata_vars?.benchmark_id?.trim() ?? "";

  return (
    <section
      data-testid="revision-summary-panel"
      className="rounded-xl border border-border/70 bg-muted/20 p-3 space-y-3"
    >
      <div className="flex items-start justify-between gap-3">
        <div className="space-y-0.5">
          <p className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">
            Revision lineage
          </p>
          <p className="text-[10px] text-muted-foreground">
            Shared benchmark package{" "}
            <span
              data-testid="revision-summary-benchmark-id"
              className="font-mono text-foreground"
            >
              {shortId(benchmarkId)}
            </span>
          </p>
        </div>
        <span
          data-testid="revision-summary-count"
          className="rounded-full border border-border/60 bg-background px-2 py-1 text-[9px] font-black uppercase tracking-widest text-muted-foreground"
        >
          {revisionEpisodes.length} revision{revisionEpisodes.length === 1 ? "" : "s"}
        </span>
      </div>

      <div
        data-testid="sidebar-lineage-badges"
        className="flex flex-wrap gap-2"
      >
        <Badge variant="outline" className="text-[9px] font-black uppercase tracking-widest">
          Benchmark {shortId(benchmarkId)}
        </Badge>
        <Badge variant="outline" className="text-[9px] font-black uppercase tracking-widest">
          {revisionEpisodes.length} Revision{revisionEpisodes.length === 1 ? "" : "s"}
        </Badge>
        <Badge
          variant="outline"
          className={cn(
            "text-[9px] font-black uppercase tracking-widest",
            selectedEpisode.metadata_vars?.is_reused ? "border-emerald-500/40 text-emerald-600" : "border-border/60 text-muted-foreground",
          )}
        >
          {selectedEpisode.metadata_vars?.is_reused ? "Reused" : "Original"}
        </Badge>
      </div>

      <div className="space-y-2">
        {revisionEpisodes.map((episode, index) => {
          const isSelected = selectedEpisode.id === episode.id;
          const parentEpisodeId = episode.metadata_vars?.prior_episode_id?.trim() ?? "";
          return (
            <button
              key={episode.id}
              type="button"
              data-testid="revision-summary-item"
              data-episode-id={episode.id}
              data-benchmark-id={benchmarkId}
              data-prior-episode-id={parentEpisodeId}
              data-is-reused={episode.metadata_vars?.is_reused ? "true" : "false"}
              data-status={episode.status}
              aria-pressed={isSelected}
              onClick={() => void onSelectEpisode(episode.id)}
              className={cn(
                "w-full rounded-lg border px-3 py-2 text-left transition-colors",
                isSelected
                  ? "border-primary bg-primary/10 shadow-sm"
                  : "border-border/60 bg-background/70 hover:bg-background",
              )}
            >
              <div className="flex items-center justify-between gap-2">
                <span className="text-[11px] font-semibold text-foreground">
                  Revision {index + 1}
                </span>
                <span
                  className={cn(
                    "text-[9px] font-black uppercase tracking-widest",
                    episode.status === EpisodeStatus.COMPLETED
                      ? "text-emerald-600"
                      : episode.status === EpisodeStatus.FAILED
                        ? "text-red-500"
                        : "text-muted-foreground",
                  )}
                >
                  {episode.status}
                </span>
              </div>
              <div className="mt-1 grid gap-0.5 text-[10px] text-muted-foreground">
                <span>
                  Parent:{" "}
                  <span
                    data-testid="revision-summary-parent"
                    className="font-mono text-foreground"
                  >
                    {parentEpisodeId ? shortId(parentEpisodeId) : "root"}
                  </span>
                </span>
                <span>
                  Reused:{" "}
                  <span
                    data-testid="revision-summary-reused"
                    className="font-mono text-foreground"
                  >
                    {episode.metadata_vars?.is_reused ? "true" : "false"}
                  </span>
                </span>
              </div>
            </button>
          );
        })}
      </div>
    </section>
  );
}
