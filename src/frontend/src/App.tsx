import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import { QueryClientProvider } from "@tanstack/react-query";
import { ReactQueryDevtools } from "@tanstack/react-query-devtools";
import { queryClient } from "@/lib/queryClient";
import "@/lib/api"; // Initialize API config
import AppLayout from "@/components/layout/AppLayout";
import HomePage from "@/pages/HomePage";
import RunDetailPage from "@/pages/RunDetailPage";
import WizardPage from "@/pages/WizardPage";

function App() {
  return (
    <QueryClientProvider client={queryClient}>
      <Router>
        <Routes>
          <Route path="/" element={<AppLayout />}>
            <Route index element={<HomePage />} />
            <Route path="runs/:id" element={<RunDetailPage />} />
            <Route path="wizard" element={<WizardPage />} />
          </Route>
        </Routes>
      </Router>
      <ReactQueryDevtools initialIsOpen={false} />
    </QueryClientProvider>
  );
}

export default App;
