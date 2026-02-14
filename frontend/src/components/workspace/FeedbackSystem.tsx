import { useState } from "react";
import { ThumbsUp, ThumbsDown, MessageSquare, CheckCircle } from "lucide-react";
import { Button } from "../ui/button";
import { cn } from "../../lib/utils";

interface FeedbackSystemProps {
    episodeId: string;
    onClose?: () => void;
}

export function FeedbackSystem({ episodeId, onClose }: FeedbackSystemProps) {
    const [rating, setRating] = useState<'positive' | 'negative' | null>(null);
    const [comment, setComment] = useState("");
    const [submitted, setSubmitted] = useState(false);
    const [selectedIssues, setSelectedIssues] = useState<string[]>([]);

    const issues = [
        "Incorrect CAD geometry",
        "Invalid code logic",
        "Hallucinated reference",
        "Poor reasoning",
        "Interface glitch"
    ];

    const handleSubmit = () => {
        console.log("Submitting feedback for", episodeId, { rating, comment, selectedIssues });
        setSubmitted(true);
        setTimeout(() => {
            if (onClose) onClose();
        }, 2000);
    };

    if (submitted) {
        return (
            <div className="p-4 bg-green-500/10 border border-green-500/20 rounded-lg flex flex-col items-center justify-center gap-2 animate-in fade-in zoom-in-95">
                <CheckCircle className="h-6 w-6 text-green-500" />
                <span className="text-xs font-bold text-green-500 uppercase tracking-widest">Feedback Received</span>
                <p className="text-[10px] text-green-500/70 text-center">Thank you for helping us improve our engineers.</p>
            </div>
        );
    }

    return (
        <div className="p-4 bg-muted/20 border border-border/50 rounded-lg space-y-4 animate-in slide-in-from-bottom-2 fade-in">
            <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                    <MessageSquare className="h-3 w-3 text-primary" />
                    <span className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">Session Feedback</span>
                </div>
                <div className="flex gap-2">
                    <Button 
                        variant="ghost" 
                        size="sm" 
                        onClick={() => setRating('positive')}
                        className={cn("h-7 px-2 gap-1.5", rating === 'positive' && "bg-green-500/10 text-green-500 hover:bg-green-500/20")}
                    >
                        <ThumbsUp className={cn("h-3 w-3", rating === 'positive' && "fill-current")} />
                    </Button>
                    <Button 
                        variant="ghost" 
                        size="sm" 
                        onClick={() => setRating('negative')}
                        className={cn("h-7 px-2 gap-1.5", rating === 'negative' && "bg-red-500/10 text-red-500 hover:bg-red-500/20")}
                    >
                        <ThumbsDown className={cn("h-3 w-3", rating === 'negative' && "fill-current")} />
                    </Button>
                </div>
            </div>

            {rating === 'negative' && (
                <div className="space-y-3 animate-in fade-in slide-in-from-top-1">
                    <div className="flex flex-wrap gap-2">
                        {issues.map(issue => (
                            <button 
                                key={issue}
                                onClick={() => {
                                    setSelectedIssues(prev => 
                                        prev.includes(issue) ? prev.filter(i => i !== issue) : [...prev, issue]
                                    )
                                }}
                                className={cn(
                                    "px-2 py-1 rounded text-[9px] font-bold uppercase tracking-wider border transition-all",
                                    selectedIssues.includes(issue) 
                                        ? "bg-red-500/10 border-red-500/30 text-red-500" 
                                        : "bg-background border-border hover:border-muted-foreground/30 text-muted-foreground"
                                )}
                            >
                                {issue}
                            </button>
                        ))}
                    </div>
                    <textarea 
                        placeholder="What went wrong? Be specific..."
                        value={comment}
                        onChange={(e) => setComment(e.target.value)}
                        className="w-full bg-background border border-border/50 rounded-md p-2 text-[11px] min-h-[60px] focus:outline-none focus:ring-1 focus:ring-primary/30"
                    />
                </div>
            )}

            {rating && (
                <div className="flex justify-end gap-2">
                    <Button 
                        size="sm" 
                        onClick={handleSubmit}
                        className="h-8 px-4 bg-primary text-primary-foreground font-black text-[9px] uppercase tracking-widest shadow-lg shadow-primary/20"
                    >
                        Submit Review
                    </Button>
                </div>
            )}
        </div>
    );
}
