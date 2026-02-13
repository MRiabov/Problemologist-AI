import { Routes, Route } from 'react-router-dom';
import AppLayout from './components/layout/AppLayout';
import EngineerWorkspace from './pages/EngineerWorkspace';
import BenchmarkGeneration from './pages/BenchmarkGeneration';
import Settings from './pages/Settings';

function App() {
  return (
    <Routes>
      <Route path="/" element={<AppLayout />}>
        <Route index element={<EngineerWorkspace />} />
        <Route path="benchmark" element={<BenchmarkGeneration />} />
        <Route path="settings" element={<Settings />} />
      </Route>
    </Routes>
  );
}

export default App;
