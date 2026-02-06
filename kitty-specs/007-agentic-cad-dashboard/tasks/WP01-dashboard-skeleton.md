---
work_package_id: "WP01"
title: "Dashboard Skeleton & Infrastructure"
lane: "planned"
dependencies: []
subtasks: ["T001", "T002", "T003", "T004", "T005"]
---

## Context

We are building the **Agentic CAD Dashboard** (Spec 007), a premium React/Vite application that serves as the UI for the Problemologist system. This dashboard will allow users to monitor agent execution, view 3D assets, and manage benchmarks.

This work package sets up the foundation of the frontend application. We will use Vite, React, TypeScript, TailwindCSS, and Shadcn/UI.

## Objective

Initialize the frontend project with the correct stack and directory structure, ensuring a "Premium" developer experience and UI foundation.

## Subtasks

### T001: Initialize Vite React Project

- **Goal**: Create a new React + TypeScript project using Vite in `src/frontend`.
- **Details**:
  - Use `npm create vite@latest src/frontend -- --template react-ts`.
  - Install core dependencies: `react`, `react-dom`, `react-router-dom`.
  - Setup `vite.config.ts` with path aliases (e.g., `@ -> src`).
- **Files**:
  - `src/frontend/package.json`
  - `src/frontend/vite.config.ts`
  - `src/frontend/tsconfig.json`

### T002: Setup TailwindCSS and Shadcn/UI

- **Goal**: Configure styling system.
- **Details**:
  - Install `tailwindcss`, `postcss`, `autoprefixer`.
  - Initialize Tailwind config: `npx tailwindcss init -p`.
  - Configure global CSS (index.css) with Tailwind directives.
  - Setup **Shadcn/UI**:
    - Run `npx shadcn-ui@latest init` (Use "New York" style, "Zinc" or "Slate" color, CSS variables: yes).
    - Install component primitives: `Button`, `Card`, `Input`, `Sheet` (for sidebar), `Separator`, `ScrollArea`.
    - Ensure components are placed in `src/components/ui`.
- **Files**:
  - `src/frontend/tailwind.config.js`
  - `src/frontend/src/index.css`
  - `src/frontend/components.json`

### T003: Configure React Router

- **Goal**: Setup client-side routing.
- **Details**:
  - Define defined routes in `src/App.tsx` (or `src/routes.tsx`):
    - `/` -> Home (Runs List) - Placeholder for now.
    - `/runs/:id` -> Run Detail - Placeholder.
    - `/wizard` -> Benchmark Wizard - Placeholder.
  - Create placeholder page components in `src/pages/`.
- **Files**:
  - `src/frontend/src/App.tsx`
  - `src/frontend/src/pages/HomePage.tsx`
  - `src/frontend/src/pages/RunDetailPage.tsx`

### T004: Setup Layout Components

- **Goal**: Create the main application shell.
- **Details**:
  - Create `AppLayout` component.
  - Implement a **Sidebar** navigation (Home, Wizard, Settings).
  - Implement a **Header** (Breadcrumbs, Theme Toggle).
  - Use Shadcn `Sheet` for mobile sidebar if needed, or a fixed sidebar for desktop.
  - Ensure responsive design.
- **Files**:
  - `src/frontend/src/components/layout/AppLayout.tsx`
  - `src/frontend/src/components/layout/Sidebar.tsx`

### T005: Configure ESLint, Prettier

- **Goal**: Enforce code quality.
- **Details**:
  - Ensure `.eslintrc.cjs` (or new flat config) is set up for React+TS.
  - Add Prettier config.
  - Ensure `npm run lint` works.
- **Files**:
  - `src/frontend/.eslintrc.cjs`
  - `src/frontend/.prettierrc`

## Implementation Guidelines

- **Directory Structure**:

  ```
  src/frontend/
  ├── src/
  │   ├── components/
  │   │   ├── ui/       # Shadcn components
  │   │   ├── layout/   # Layout components
  │   ├── pages/        # Page components
  │   ├── lib/          # Utilities (utils.ts for shadcn)
  │   ├── App.tsx
  │   └── main.tsx
  ```

- **Aesthetics**: Follow the "Premium" guideline. Use subtle borders, nice typography (Inter), and good spacing.

## Validation (Definition of Done)

- [ ] `npm install` runs successfully in `src/frontend`.
- [ ] `npm run dev` starts the server.
- [ ] Browser shows the App Shell with Sidebar and correct routing placeholders.
- [ ] Shadcn components render correctly with styles.
- [ ] User can navigate between Home and Wizard placeholders.
