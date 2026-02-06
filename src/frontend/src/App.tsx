import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import AppLayout from "@/components/layout/AppLayout";
import HomePage from "@/pages/HomePage";
import RunDetailPage from "@/pages/RunDetailPage";
import WizardPage from "@/pages/WizardPage";

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<AppLayout />}>
          <Route index element={<HomePage />} />
          <Route path="runs/:id" element={<RunDetailPage />} />
          <Route path="wizard" element={<WizardPage />} />
        </Route>
      </Routes>
    </Router>
  );
}

export default App;
