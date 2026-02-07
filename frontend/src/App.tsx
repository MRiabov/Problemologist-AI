import { Routes, Route } from 'react-router-dom';
import AppLayout from './components/layout/AppLayout';
import EngineerWorkspace from './pages/EngineerWorkspace';
import BenchmarkGeneration from './pages/BenchmarkGeneration';
import { EpisodeProvider } from './context/EpisodeContext';
import { ConnectionProvider } from './context/ConnectionContext';

function App() {
  return (
    <ConnectionProvider>
      <EpisodeProvider>
        <Routes>
          <Route path="/" element={<AppLayout />}>
            <Route index element={<EngineerWorkspace />} />
            <Route path="benchmark" element={<BenchmarkGeneration />} />
          </Route>
        </Routes>
      </EpisodeProvider>
    </ConnectionProvider>
  );
}

export default App;
