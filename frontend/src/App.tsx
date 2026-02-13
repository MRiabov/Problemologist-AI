import { Routes, Route } from 'react-router-dom';
import AppLayout from './components/layout/AppLayout';
import EngineerWorkspace from './pages/EngineerWorkspace';
import BenchmarkGeneration from './pages/BenchmarkGeneration';
import Settings from './pages/Settings';
import { EpisodeProvider } from './context/EpisodeContext';
import { ConnectionProvider } from './context/ConnectionContext';
import { ThemeProvider } from './context/ThemeContext';

function App() {
  return (
    <ThemeProvider>
      <ConnectionProvider>
        <EpisodeProvider>
          <Routes>
            <Route path="/" element={<AppLayout />}>
              <Route index element={<EngineerWorkspace />} />
              <Route path="benchmark" element={<BenchmarkGeneration />} />
              <Route path="settings" element={<Settings />} />
            </Route>
          </Routes>
        </EpisodeProvider>
      </ConnectionProvider>
    </ThemeProvider>
  );
}

export default App;
