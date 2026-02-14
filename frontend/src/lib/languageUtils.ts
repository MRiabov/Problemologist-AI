
/**
 * Maps file extensions to react-syntax-highlighter compatible languages.
 * @param filename The name of the file (e.g., "script.py")
 * @param fallback The default language if no match is found (default: 'text')
 * @returns The detected language string
 */
export function detectLanguage(filename: string, fallback: string = 'text'): string {
    if (!filename) return fallback;
    
    const ext = filename.split('.').pop()?.toLowerCase();
    
    switch (ext) {
        case 'js':
        case 'jsx':
        case 'mjs':
        case 'cjs':
            return 'javascript';
        case 'ts':
        case 'tsx':
            return 'typescript';
        case 'py':
        case 'pyw':
            return 'python';
        case 'html':
        case 'htm':
            return 'html';
        case 'css':
        case 'scss':
        case 'sass':
        case 'less':
            return 'css';
        case 'json':
        case 'jsonc':
            return 'json';
        case 'md':
        case 'markdown':
            return 'markdown';
        case 'yaml':
        case 'yml':
            return 'yaml';
        case 'xml':
        case 'svg':
            return 'xml';
        case 'sh':
        case 'bash':
        case 'zsh':
            return 'bash';
        case 'sql':
            return 'sql';
        case 'java':
            return 'java';
        case 'c':
        case 'h':
            return 'c';
        case 'cpp':
        case 'hpp':
        case 'cc':
            return 'cpp';
        case 'cs':
            return 'csharp';
        case 'go':
            return 'go';
        case 'rs':
        case 'rust':
            return 'rust';
        case 'php':
            return 'php';
        case 'rb':
            return 'ruby';
        case 'lua':
            return 'lua';
        case 'dockerfile':
            return 'dockerfile';
        case 'ini':
        case 'toml':
        case 'cfg':
        case 'conf':
            return 'ini';
        default:
            // Check for specific filenames
            if (filename.toLowerCase() === 'dockerfile') return 'dockerfile';
            if (filename.toLowerCase() === 'makefile') return 'makefile';
            return fallback;
    }
}
