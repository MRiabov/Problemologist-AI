import { AssetType } from "../../api/generated/models/AssetType";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";

export type ArtifactSelectionContext = {
  episodeType?: string | null;
  isBenchmarkRoute?: boolean;
  plan?: string | null;
  assets?: AssetResponse[];
};

const MEDIA_IMAGE_EXTENSIONS = new Set(["png", "jpg", "jpeg", "webp", "gif"]);
const MEDIA_VIDEO_EXTENSIONS = new Set(["mp4", "webm", "mov"]);

export const getAssetFileName = (asset: AssetResponse): string => {
  return asset.s3_path.split("/").pop() || asset.s3_path;
};

const getAssetExtension = (asset: AssetResponse): string => {
  const fileName = getAssetFileName(asset).toLowerCase();
  const dotIndex = fileName.lastIndexOf(".");
  return dotIndex >= 0 ? fileName.slice(dotIndex + 1) : "";
};

const getAssetTimestamp = (asset: AssetResponse): number => {
  const parsed = Date.parse(asset.created_at);
  return Number.isFinite(parsed) ? parsed : 0;
};

export const isImageAsset = (asset: AssetResponse): boolean => {
  return (
    asset.asset_type === AssetType.IMAGE ||
    MEDIA_IMAGE_EXTENSIONS.has(getAssetExtension(asset))
  );
};

export const isVideoAsset = (asset: AssetResponse): boolean => {
  return (
    asset.asset_type === AssetType.VIDEO ||
    MEDIA_VIDEO_EXTENSIONS.has(getAssetExtension(asset))
  );
};

export const isRenderEvidenceAsset = (asset: AssetResponse): boolean => {
  const fileName = getAssetFileName(asset).toLowerCase();
  return (
    fileName === "simulation_result.json" ||
    fileName === "validation_results.json" ||
    isImageAsset(asset) ||
    isVideoAsset(asset) ||
    asset.s3_path.toLowerCase().includes("/renders/") ||
    asset.s3_path.toLowerCase().startsWith("renders/")
  );
};

const getEvidencePriority = (asset: AssetResponse): number => {
  const fileName = getAssetFileName(asset).toLowerCase();
  if (fileName === "simulation_result.json") return 0;
  if (fileName === "validation_results.json") return 1;
  if (asset.asset_type === AssetType.VIDEO) return 2;
  if (asset.asset_type === AssetType.IMAGE) return 3;
  if (asset.s3_path.toLowerCase().includes("/renders/")) return 4;
  if (asset.s3_path.toLowerCase().startsWith("renders/")) return 4;
  if (MEDIA_VIDEO_EXTENSIONS.has(getAssetExtension(asset))) return 5;
  if (MEDIA_IMAGE_EXTENSIONS.has(getAssetExtension(asset))) return 6;
  return 10;
};

export const getLatestSolutionEvidenceAsset = (
  assets: AssetResponse[] = []
): AssetResponse | null => {
  const candidates = assets
    .filter(isRenderEvidenceAsset)
    .slice()
    .sort((a, b) => {
      const priorityDelta = getEvidencePriority(a) - getEvidencePriority(b);
      if (priorityDelta !== 0) return priorityDelta;

      const timestampDelta = getAssetTimestamp(b) - getAssetTimestamp(a);
      if (timestampDelta !== 0) return timestampDelta;

      const idDelta = b.id - a.id;
      if (idDelta !== 0) return idDelta;

      return getAssetFileName(a).localeCompare(getAssetFileName(b));
    });

  return candidates[0] ?? null;
};

export const getDefaultArtifactId = ({
  episodeType,
  isBenchmarkRoute = false,
  plan,
  assets = [],
}: ArtifactSelectionContext): string | null => {
  const solutionEvidence = getLatestSolutionEvidenceAsset(assets);
  const isBenchmarkEpisode = episodeType === "benchmark" || isBenchmarkRoute;

  if (isBenchmarkEpisode) {
    return plan ? "plan" : solutionEvidence?.id.toString() ?? assets[0]?.id.toString() ?? null;
  }

  return (
    solutionEvidence?.id.toString() ??
    (plan ? "plan" : assets[0]?.id.toString() ?? null)
  );
};

export const getArtifactSelectionDescriptor = (
  asset: AssetResponse | null
): {
  id: string | null;
  name: string | null;
  path: string | null;
  assetType: string | null;
} => {
  if (!asset) {
    return {
      id: null,
      name: null,
      path: null,
      assetType: null,
    };
  }

  return {
    id: asset.id.toString(),
    name: getAssetFileName(asset),
    path: asset.s3_path,
    assetType: asset.asset_type,
  };
};
