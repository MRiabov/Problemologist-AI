import { Routes, Route } from 'react-router-dom';
import AppLayout from './components/layout/AppLayout';
import EngineerWorkspace from './pages/EngineerWorkspace';
import IdeDashboard from './pages/IdeDashboard';
import BenchmarkGeneration from './pages/BenchmarkGeneration';

function App() {
  return (
    <Routes>
      <Route path="/" element={<AppLayout />}>
        <Route index element={<EngineerWorkspace />} />
        <Route path="ide" element={<IdeDashboard />} />
        <Route path="benchmark" element={<BenchmarkGeneration />} />
      </Route>
    </Routes>
  );
}

export default App;
