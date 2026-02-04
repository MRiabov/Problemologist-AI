import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import IdeDashboard from './screens/IdeDashboard';
import EngineerWorkspace from './screens/EngineerWorkspace';
import BenchmarkGenerator from './screens/BenchmarkGenerator';

function App() {
  return (
    <Router>
      <div className="h-screen flex flex-col bg-background-dark text-white font-display overflow-hidden">
        <Routes>
          <Route path="/" element={<IdeDashboard />} />
          <Route path="/workspace" element={<EngineerWorkspace />} />
          <Route path="/benchmark" element={<BenchmarkGenerator />} />
        </Routes>
      </div>
    </Router>
  );
}

export default App;
