import { 
  VscMarkdown, 
  VscJson, 
  VscTasklist, 
  VscSettings, 
  VscPlayCircle, 
  VscFileMedia,
  VscCode
} from "react-icons/vsc";
import { SiPython } from "react-icons/si";
import { AssetType } from "../api/generated/models/AssetType";

export const getFileIconInfo = (name: string, type?: string) => {
  const fileName = name.toLowerCase();
  
  if (fileName === 'plan.md' || type === AssetType.MARKDOWN) return { icon: VscTasklist, color: "#28A745" };
  if (fileName.endsWith('.py') || type === AssetType.PYTHON) return { icon: SiPython, color: "#3776AB" };
  if (fileName.endsWith('.md') || type === AssetType.MARKDOWN) return { icon: VscMarkdown, color: "#007ACC" };
  if (fileName.endsWith('.json') || fileName.endsWith('.mjcf') || type === AssetType.MJCF || type === 'json') return { icon: VscJson, color: "#FBC02D" };
  if (fileName.endsWith('.yaml') || fileName.endsWith('.yml')) return { icon: VscSettings, color: "#CB2431" };
  if (type === AssetType.VIDEO) return { icon: VscPlayCircle, color: "#E44D26" };
  if (type === AssetType.IMAGE) return { icon: VscFileMedia, color: "#47A248" };
  
  return { icon: VscCode, color: "#858585" };
};
