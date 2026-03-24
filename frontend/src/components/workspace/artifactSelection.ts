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
const CAD_RENDERABLE_EXTENSIONS = new Set(["glb", "gltf", "obj", "stl"]);
const CAD_SOURCE_EXTENSIONS = new Set(["step", "stp"]);
const CAD_MODEL_EXTENSIONS = new Set([
  ...CAD_RENDERABLE_EXTENSIONS,
  ...CAD_SOURCE_EXTENSIONS,
]);
const SOLUTION_EVIDENCE_FILENAMES = [
  "simulation_result.json",
  "validation_results.json",
];

const pickPreferredAsset = (assets: AssetResponse[]): AssetResponse | null => {
  if (assets.length === 0) {
    return null;
  }

  return assets[0] ?? null;
};

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

export const isModelAsset = (asset: AssetResponse): boolean => {
  return (
    asset.asset_type === AssetType.GLB ||
    asset.asset_type === AssetType.STL ||
    asset.asset_type === AssetType.STEP ||
    CAD_MODEL_EXTENSIONS.has(getAssetExtension(asset))
  );
};

export const isRenderableModelAsset = (asset: AssetResponse): boolean => {
  return (
    asset.asset_type === AssetType.GLB ||
    asset.asset_type === AssetType.STL ||
    CAD_RENDERABLE_EXTENSIONS.has(getAssetExtension(asset))
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
  return pickPreferredAsset(assets.filter(predicate));
};

const getLatestAssetByFileName = (
  assets: AssetResponse[] = [],
  fileNames: string[],
): AssetResponse | null => {
  const normalizedTargets = fileNames.map((fileName) => fileName.toLowerCase());
  for (const fileName of normalizedTargets) {
    const match = pickPreferredAsset(
      assets.filter((asset) => getAssetFileName(asset).toLowerCase() === fileName),
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
  const modelAsset =
    getLatestMatchingAsset(assets, isRenderableModelAsset) ??
    getLatestMatchingAsset(assets, isModelAsset);
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

export const getLatestModelAsset = (
  assets: AssetResponse[] = [],
): AssetResponse | null => {
  return (
    getLatestMatchingAsset(assets, isRenderableModelAsset) ??
    getLatestMatchingAsset(assets, isModelAsset)
  );
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

  // The controller already orders assets by the intended revision precedence,
  // so the first matching item is the authoritative one.
  if (isBenchmarkEpisode) {
    return plan ? "plan" : solutionEvidence?.id.toString() ?? pickPreferredAsset(assets)?.id.toString() ?? null;
  }

  return (
    solutionEvidence?.id.toString() ??
    (plan ? "plan" : pickPreferredAsset(assets)?.id.toString() ?? null)
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
