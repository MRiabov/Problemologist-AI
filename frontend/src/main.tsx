import React from 'react'
import ReactDOM from 'react-dom/client'
import App from './App.tsx'
import './index.css'
import { BrowserRouter } from 'react-router-dom'
import { OpenAPI } from './api/generated/core/OpenAPI'
import { EpisodeProvider } from './context/EpisodeContext'
import { ConnectionProvider } from './context/ConnectionContext'
import { ThemeProvider } from './context/ThemeContext'

const API_BASE = import.meta.env.VITE_API_URL || '/api';
OpenAPI.BASE = API_BASE;

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <ThemeProvider>
      <ConnectionProvider>
        <EpisodeProvider>
          <BrowserRouter>
            <App />
          </BrowserRouter>
        </EpisodeProvider>
      </ConnectionProvider>
    </ThemeProvider>
  </React.StrictMode>,
)
