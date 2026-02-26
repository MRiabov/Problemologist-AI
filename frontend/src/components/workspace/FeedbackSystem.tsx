import { useState } from "react";
import { CheckCircle, ThumbsUp, ThumbsDown, AlertTriangle } from "lucide-react";
import { Button } from "../ui/button";
import { submitTraceFeedback } from "../../api/client";
import { cn } from "../../lib/utils";

interface FeedbackSystemProps {
    episodeId: string;
    traceId: number;
    initialScore: number;
    onClose?: () => void;
}

const FEEDBACK_TOPICS = [
    "Misinterpretation",
    "Doesn't follow instructions",
    "Technical Error",
    "Poor Design",
    "Other"
];

export function FeedbackSystem({ episodeId, traceId, initialScore, onClose }: FeedbackSystemProps) {
    const [comment, setComment] = useState("");
    const [score, setScore] = useState(initialScore);
    const [topic, setTopic] = useState<string | null>(null);
    const [submitted, setSubmitted] = useState(false);
    const [isSubmitting, setIsSubmitting] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const handleSubmit = async () => {
        setIsSubmitting(true);
        setError(null);
        try {
            const combinedComment = topic ? `[${topic}] ${comment}` : comment;
            await submitTraceFeedback(episodeId, traceId, score, combinedComment);
            setSubmitted(true);
            setTimeout(() => {
                if (onClose) onClose();
            }, 2000);
        } catch (e) {
            console.error("Failed to submit feedback", e);
            setError("Failed to submit feedback. Please try again.");
        } finally {
            setIsSubmitting(false);
        }
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
        <div className="fixed inset-0 z-50 flex items-center justify-center p-4 min-h-screen">
            <div 
                className="absolute inset-0 bg-background/60 backdrop-blur-[2px]" 
                onClick={onClose}
            />
            <div 
                data-testid="feedback-modal"
                className="bg-background border border-border shadow-2xl rounded-2xl p-6 max-w-lg w-full relative animate-in fade-in zoom-in-95 duration-200"
            >
                <div className="flex items-center justify-between mb-6">
                    <h2 className="text-lg font-bold text-foreground uppercase tracking-tight">Agent Feedback</h2>
                    <div className="flex items-center gap-1 bg-muted/50 p-1 rounded-xl border border-border/50">
                        <button
                            data-testid="modal-thumbs-up"
                            onClick={() => setScore(1)}
                            className={cn(
                                "p-2 rounded-lg transition-all",
                                score === 1 ? "bg-background text-green-500 shadow-sm" : "text-muted-foreground hover:text-foreground"
                            )}
                        >
                            <ThumbsUp className="h-4 w-4" />
                        </button>
                        <button
                            data-testid="modal-thumbs-down"
                            onClick={() => setScore(0)}
                            className={cn(
                                "p-2 rounded-lg transition-all",
                                score === 0 ? "bg-background text-red-500 shadow-sm" : "text-muted-foreground hover:text-foreground"
                            )}
                        >
                            <ThumbsDown className="h-4 w-4" />
                        </button>
                    </div>
                </div>
                
                <div className="space-y-6">
                    <div className="space-y-3">
                        <label className="text-[11px] font-black uppercase tracking-widest text-muted-foreground">
                            What went wrong?
                        </label>
                        <div className="flex flex-wrap gap-2">
                            {FEEDBACK_TOPICS.map(t => (
                                <button
                                    key={t}
                                    onClick={() => setTopic(t)}
                                    className={cn(
                                        "px-3 py-1.5 rounded-full text-[11px] font-bold border transition-all",
                                        topic === t
                                            ? "bg-primary/10 border-primary text-primary"
                                            : "bg-background border-border text-muted-foreground hover:border-primary/50"
                                    )}
                                >
                                    {t}
                                </button>
                            ))}
                        </div>
                    </div>

                    <div className="space-y-3">
                        <label className="text-[11px] font-black uppercase tracking-widest text-muted-foreground">
                            Additional Details (Optional)
                        </label>
                        <textarea 
                            placeholder={score === 1 ? "What was satisfying about this response?" : "How can the agent improve?"}
                            value={comment}
                            onChange={(e) => setComment(e.target.value)}
                            className="w-full bg-muted/30 border border-border rounded-xl p-4 text-[13px] min-h-[100px] focus:ring-2 focus:ring-primary/10 focus:border-primary/30 transition-all outline-none resize-none"
                        />
                    </div>

                    {error && (
                        <div className="p-3 bg-red-500/10 border border-red-500/20 rounded-lg flex items-center gap-2 text-red-500 text-[11px] font-bold">
                            <AlertTriangle className="h-3.5 w-3.5" />
                            {error}
                        </div>
                    )}

                    <p className="text-[10px] text-muted-foreground/60 leading-relaxed italic">
                        Submitting this report will send the conversation segment to our evaluation system to help improve agent reasoning.
                    </p>

                    <div className="flex justify-end gap-3 pt-2">
                        <Button 
                            variant="outline"
                            onClick={onClose}
                            disabled={isSubmitting}
                            className="h-9 px-6 rounded-xl border-border hover:bg-muted font-bold text-[11px] uppercase tracking-widest"
                        >
                            Cancel
                        </Button>
                        <Button 
                            onClick={handleSubmit}
                            disabled={isSubmitting}
                            className="h-9 px-6 rounded-xl bg-foreground text-background hover:opacity-90 font-bold text-[11px] uppercase tracking-widest transition-all shadow-lg shadow-foreground/10"
                        >
                            {isSubmitting ? "Submitting..." : "Send Feedback"}
                        </Button>
                    </div>
                </div>
            </div>
        </div>
    );
}
