import { useState } from "react";
import { CheckCircle } from "lucide-react";
import { Button } from "../ui/button";

interface FeedbackSystemProps {
    episodeId: string;
    onClose?: () => void;
}

export function FeedbackSystem({ episodeId, onClose }: FeedbackSystemProps) {
    const [comment, setComment] = useState("");
    const [submitted, setSubmitted] = useState(false);

    const handleSubmit = () => {
        console.log("Submitting feedback for", episodeId, { comment });
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
        <div className="fixed inset-0 z-50 flex items-center justify-center p-4 min-h-screen">
            <div 
                className="absolute inset-0 bg-background/60 backdrop-blur-[2px]" 
                onClick={onClose}
            />
            <div className="bg-background border border-border shadow-2xl rounded-2xl p-6 max-w-lg w-full relative animate-in fade-in zoom-in-95 duration-200">
                <h2 className="text-lg font-semibold text-foreground mb-4">Feedback</h2>
                
                <div className="space-y-4">
                    <div className="space-y-2">
                        <label className="text-[13px] text-muted-foreground">
                            Please provide details: (optional)
                        </label>
                        <textarea 
                            placeholder="What was satisfying about this response?"
                            value={comment}
                            onChange={(e) => setComment(e.target.value)}
                            className="w-full bg-background border border-border rounded-xl p-4 text-[14px] min-h-[120px] focus:ring-1 focus:ring-primary/20 focus:border-primary/30 transition-all outline-none resize-none"
                        />
                    </div>

                    <p className="text-[12px] text-muted-foreground/60 leading-relaxed">
                        Submitting this report will send the entire current conversation to the evaluation system for future improvements to our models. <span className="underline cursor-pointer hover:text-muted-foreground transition-colors">Learn More</span>
                    </p>

                    <div className="flex justify-end gap-3 pt-4">
                        <Button 
                            variant="outline"
                            onClick={onClose}
                            className="h-9 px-5 rounded-xl border-border hover:bg-muted font-medium text-[13px]"
                        >
                            Cancel
                        </Button>
                        <Button 
                            onClick={handleSubmit}
                            className="h-9 px-5 rounded-xl bg-foreground text-background hover:opacity-90 font-medium text-[13px] transition-all"
                        >
                            Submit
                        </Button>
                    </div>
                </div>
            </div>
        </div>
    );
}
