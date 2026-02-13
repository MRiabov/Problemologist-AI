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

export const getFileIconInfo = (name: string, type?: string) => {
  const fileName = name.toLowerCase();
  
  if (fileName === 'plan.md' || type === 'plan') return { icon: VscTasklist, color: "#28A745" };
  if (fileName.endsWith('.py') || type === 'python') return { icon: SiPython, color: "#3776AB" };
  if (fileName.endsWith('.md') || type === 'markdown') return { icon: VscMarkdown, color: "#007ACC" };
  if (fileName.endsWith('.json') || fileName.endsWith('.mjcf') || type === 'mjcf' || type === 'json') return { icon: VscJson, color: "#FBC02D" };
  if (fileName.endsWith('.yaml') || fileName.endsWith('.yml')) return { icon: VscSettings, color: "#CB2431" };
  if (type === 'video') return { icon: VscPlayCircle, color: "#E44D26" };
  if (type === 'image') return { icon: VscFileMedia, color: "#47A248" };
  
  return { icon: VscCode, color: "#858585" };
};
