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
const SOLUTION_EVIDENCE_FILENAMES = [
  "simulation_result.json",
  "validation_results.json",
];

export const getAssetFileName = (asset: AssetResponse): string => {
  return asset.s3_path.split("/").pop() || asset.s3_path;
};

const getAssetExtension = (asset: AssetResponse): string => {
  const fileName = getAssetFileName(asset).toLowerCase();
  const dotIndex = fileName.lastIndexOf(".");
  return dotIndex >= 0 ? fileName.slice(dotIndex + 1) : "";
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

export const isSolutionEvidenceAsset = (asset: AssetResponse): boolean => {
  const fileName = getAssetFileName(asset).toLowerCase();
  return (
    SOLUTION_EVIDENCE_FILENAMES.includes(fileName) ||
    isImageAsset(asset) ||
    asset.s3_path.toLowerCase().includes("/renders/") ||
    asset.s3_path.toLowerCase().startsWith("renders/")
  );
};

export const getLatestMatchingAsset = (
  assets: AssetResponse[] = [],
  predicate: (asset: AssetResponse) => boolean,
): AssetResponse | null => {
  return assets.find(predicate) ?? null;
};

const getLatestAssetByFileName = (
  assets: AssetResponse[] = [],
  fileNames: string[],
): AssetResponse | null => {
  const normalizedTargets = fileNames.map((fileName) => fileName.toLowerCase());
  for (const fileName of normalizedTargets) {
    const match = assets.find(
      (asset) => getAssetFileName(asset).toLowerCase() === fileName,
    );
    if (match) {
      return match;
    }
  }
  return null;
};

export const getLatestMediaBundle = (
  assets: AssetResponse[] = [],
): {
  solutionEvidenceAsset: AssetResponse | null;
  videoAsset: AssetResponse | null;
  modelAsset: AssetResponse | null;
  heatmapAsset: AssetResponse | null;
} => {
  const solutionEvidenceAsset =
    getLatestAssetByFileName(assets, SOLUTION_EVIDENCE_FILENAMES) ??
    getLatestMatchingAsset(
      assets,
      (asset) =>
        isImageAsset(asset) ||
        asset.s3_path.toLowerCase().includes("/renders/") ||
        asset.s3_path.toLowerCase().startsWith("renders/"),
    ) ??
    getLatestMatchingAsset(assets, isVideoAsset);
  const videoAsset = getLatestMatchingAsset(
    assets,
    (asset) => asset.asset_type === AssetType.VIDEO || MEDIA_VIDEO_EXTENSIONS.has(getAssetExtension(asset)),
  );
  const modelAsset = getLatestMatchingAsset(assets, (asset) => asset.asset_type === AssetType.GLB);
  const heatmapAsset = getLatestMatchingAsset(
    assets,
    (asset) =>
      isImageAsset(asset) &&
      asset.s3_path.toLowerCase().includes("stress_"),
  );

  return {
    solutionEvidenceAsset,
    videoAsset,
    modelAsset,
    heatmapAsset,
  };
};

export const getLatestSolutionEvidenceAsset = (
  assets: AssetResponse[] = []
): AssetResponse | null => {
  return (
    getLatestAssetByFileName(assets, SOLUTION_EVIDENCE_FILENAMES) ??
    getLatestMatchingAsset(
      assets,
      (asset) =>
        isImageAsset(asset) ||
        asset.s3_path.toLowerCase().includes("/renders/") ||
        asset.s3_path.toLowerCase().startsWith("renders/"),
    ) ??
    getLatestMatchingAsset(assets, isVideoAsset)
  );
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
