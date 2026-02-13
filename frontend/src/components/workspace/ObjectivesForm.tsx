import React from 'react';
import { Input } from '../ui/input';
import { Button } from '../ui/button';
import { BenchmarkObjectives } from '../../api/client';

interface ObjectivesFormProps {
    objectives: BenchmarkObjectives;
    onChange: (objectives: BenchmarkObjectives) => void;
    onUpdate?: () => void;
    showUpdate?: boolean;
    disabled?: boolean;
}

export function ObjectivesForm({ objectives, onChange, onUpdate, showUpdate, disabled }: ObjectivesFormProps) {
    const handleChange = (field: keyof BenchmarkObjectives, value: string) => {
        const numVal = value ? parseFloat(value) : undefined;
        onChange({ ...objectives, [field]: numVal });
    };

    return (
        <div className="p-3 bg-muted/40 border-t border-border/50 grid grid-cols-3 gap-3 animate-in slide-in-from-bottom-2 fade-in duration-200">
            <div className="flex flex-col gap-1.5">
                <label className="text-[10px] uppercase tracking-wider font-semibold text-muted-foreground/70">Max Cost ($)</label>
                <Input
                    type="number"
                    value={objectives.max_cost ?? ''}
                    onChange={(e) => handleChange('max_cost', e.target.value)}
                    placeholder="50.00"
                    disabled={disabled}
                    className="h-8 text-xs bg-background/50"
                />
            </div>
            <div className="flex flex-col gap-1.5">
                <label className="text-[10px] uppercase tracking-wider font-semibold text-muted-foreground/70">Max Weight (kg)</label>
                <Input
                    type="number"
                    step="0.1"
                    value={objectives.max_weight ?? ''}
                    onChange={(e) => handleChange('max_weight', e.target.value)}
                    placeholder="1.2"
                    disabled={disabled}
                    className="h-8 text-xs bg-background/50"
                />
            </div>
            <div className="flex flex-col gap-1.5">
                <label className="text-[10px] uppercase tracking-wider font-semibold text-muted-foreground/70">Count</label>
                <Input
                    type="number"
                    value={objectives.target_quantity ?? ''}
                    onChange={(e) => handleChange('target_quantity', e.target.value)}
                    placeholder="1000"
                    disabled={disabled}
                    className="h-8 text-xs bg-background/50"
                />
            </div>
            {showUpdate && onUpdate && (
                <div className="col-span-3 flex justify-end pt-1">
                    <Button
                        size="sm"
                        variant="secondary"
                        onClick={onUpdate}
                        disabled={disabled}
                        className="h-7 text-xs px-3"
                    >
                        Update Objectives
                    </Button>
                </div>
            )}
        </div>
    );
}
