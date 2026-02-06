# Dashboard Quickstart

## 1. Development Mode

The dashboard requires both the backend (FastAPI) and frontend (Vite) to be running.

### Backend

```bash
uv run python -m src.dashboard.backend.main
```

### Frontend

```bash
cd src/dashboard/frontend
npm install
npm run dev
```

The app will be available at `http://localhost:5173`.

## 2. Production Deployment

The dashboard is automatically deployed to **Vercel** (frontend) and **Railway** (backend) as part of the CI/CD pipeline.

## 3. Features

- **Live Reasoning Stream**: Observe the agent's thoughts and tool calls in real-time.
- **3D Interactive Viewport**: Inspect meshes and violations.
- **Artifact Timeline**: Step through the history of an episode.
