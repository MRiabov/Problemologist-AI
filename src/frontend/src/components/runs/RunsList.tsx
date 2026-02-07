import { useNavigate } from 'react-router-dom';
import { useRuns } from '@/hooks/api/useRuns';
import { Button } from '@/components/ui/button';

export function RunsList() {
  const navigate = useNavigate();
  const { data: runs, isLoading, error } = useRuns();

  if (isLoading) {
    return <div className="p-4 text-center">Loading runs...</div>;
  }

  if (error) {
    return <div className="p-4 text-center text-red-500">Error loading runs: {error.message}</div>;
  }

  if (!runs || runs.length === 0) {
    return <div className="p-4 text-center text-muted-foreground">No runs found.</div>;
  }

  return (
    <div className="rounded-md border">
      <table className="w-full text-sm">
        <thead className="border-b bg-muted/50">
          <tr className="text-left">
            <th className="p-4 font-medium">ID</th>
            <th className="p-4 font-medium">Task</th>
            <th className="p-4 font-medium">Status</th>
            <th className="p-4 font-medium">Created At</th>
            <th className="p-4 font-medium">Actions</th>
          </tr>
        </thead>
        <tbody>
          {runs.map((run) => (
            <tr key={run.id} className="border-b transition-colors hover:bg-muted/50">
              <td className="p-4 font-mono">{run.id}</td>
              <td className="p-4 truncate max-w-[200px]">{run.task}</td>
              <td className="p-4">
                <span className={`inline-flex items-center rounded-full px-2.5 py-0.5 text-xs font-semibold transition-colors focus:outline-none focus:ring-2 focus:ring-ring focus:ring-offset-2 ${
                  run.status === 'running' ? 'bg-blue-500/10 text-blue-500' :
                  run.status === 'completed' ? 'bg-green-500/10 text-green-500' :
                  run.status === 'failed' ? 'bg-red-500/10 text-red-500' :
                  'bg-gray-500/10 text-gray-500'
                }`}>
                  {run.status}
                </span>
              </td>
              <td className="p-4">{new Date(run.created_at).toLocaleString()}</td>
              <td className="p-4">
                <Button 
                  variant="outline" 
                  size="sm"
                  onClick={() => navigate(`/runs/${run.id}`)}
                >
                  View
                </Button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
