import { defineConfig } from 'vitest/config'
import react from '@vitejs/plugin-react'
import path from "path"

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: [
      { find: '@', replacement: path.resolve(__dirname, 'src') },
    ],
  },
  test: {
    globals: true,
    environment: 'jsdom',
    setupFiles: './src/test/setup.ts',
  },
  server: {
    port: 15173,
    proxy: {
      '/api/benchmark/build': {
        target: 'http://localhost:18001',
        changeOrigin: true,
      },
      '/api': {
        target: 'http://localhost:18000',
        changeOrigin: true,
        ws: true,
      },
    },
  },
})
