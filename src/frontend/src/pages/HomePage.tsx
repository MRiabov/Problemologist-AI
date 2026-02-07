import { RunsList } from "@/components/runs/RunsList";

export default function HomePage() {
  return (
    <div className="container mx-auto py-8">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold tracking-tight">Agent Runs</h1>
          <p className="text-muted-foreground mt-2">
            Monitor and manage active and past agent episodes.
          </p>
        </div>
      </div>
      <RunsList />
    </div>
  );
}