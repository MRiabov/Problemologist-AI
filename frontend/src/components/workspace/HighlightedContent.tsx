import { memo } from 'react';
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { vscDarkPlus, vs } from "react-syntax-highlighter/dist/esm/styles/prism";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import type { ContextItem } from "../../context/EpisodeContext";

interface HighlightedContentProps {
  content: string;
  language?: string;
  theme: string;
  assets: AssetResponse[] | undefined;
  addToContext: (item: ContextItem) => void;
  setActiveArtifactId: (id: string | null) => void;
}

// Internal helper for rendering highlighted content
const HighlightedContent = memo(({
  content,
  language = 'text',
  theme,
  assets,
  addToContext,
  setActiveArtifactId
}: HighlightedContentProps) => {

        if (!content) return null;

        // Custom renderer for mentions in markdown/text
        const renderWithMentions = (text: string) => {
            const regex = /(@([a-zA-Z0-9_\-.]+)(?::L?(\d+)(?:-L?(\d+))?)?)/g;
            const parts = [];
            let lastIndex = 0;
            let match;

            while ((match = regex.exec(text)) !== null) {
                const [full, , name, start, end] = match;
                parts.push(text.substring(lastIndex, match.index));

                parts.push(
                    <button
                        key={match.index}
                        className="text-primary hover:underline font-mono font-bold px-1 rounded bg-primary/10"
                        onClick={() => {
                            const asset = assets?.find(a => a.s3_path.endsWith(name) || a.s3_path === name);
                            if (asset) {
                                setActiveArtifactId(asset.id.toString());
                                if (start) {
                                    addToContext({
                                        id: `steer-${asset.id}-${start}-${end || start}`,
                                        type: 'code',
                                        label: full.substring(1),
                                        metadata: {
                                            path: asset.s3_path,
                                            start: parseInt(start),
                                            end: end ? parseInt(end) : parseInt(start)
                                        }
                                    });
                                }
                            } else {
                                // Assume part mention
                                addToContext({
                                    id: `cad-${name}`,
                                    type: 'cad',
                                    label: name,
                                    metadata: { part: name }
                                });
                            }
                        }}
                    >
                        {full}
                    </button>
                );
                lastIndex = regex.lastIndex;
            }
            parts.push(text.substring(lastIndex));
            return parts;
        };

        // Basic detection for JSON strings
        const lang = (content.trim().startsWith('{') || content.trim().startsWith('[')) ? 'json' : language;

        if (lang === 'markdown' || lang === 'text') {
            return (
                <div className="text-[13px] leading-relaxed text-foreground/90 whitespace-pre-wrap py-1">
                    {renderWithMentions(content)}
                </div>
            );
        }

        return (
                <SyntaxHighlighter
                        language={lang}
                        style={theme === 'dark' ? vscDarkPlus : vs}
                        customStyle={{
                                margin: 0,
                                padding: '0.5rem',
                                background: theme === 'dark' ? 'rgba(0,0,0,0.2)' : 'rgba(0,0,0,0.05)',
                                fontSize: '10px',
                                borderRadius: '4px',
                                border: theme === 'dark' ? '1px solid rgba(255,255,255,0.05)' : '1px solid rgba(0,0,0,0.05)'
                        }}
                        wrapLines={true}
                        wrapLongLines={true}
                >
                        {content}
                </SyntaxHighlighter>
        );
});

HighlightedContent.displayName = 'HighlightedContent';

export { HighlightedContent };
