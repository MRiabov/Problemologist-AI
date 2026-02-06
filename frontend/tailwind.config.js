/** @type {import('tailwindcss').Config} */
export default {
  darkMode: "class",
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        "primary": "#1e94f6",
        "primary-dark": "#167acc",
        "background-light": "#f5f7f8",
        "background-dark": "#0a0f14",
        "panel-dark": "#121921",
        "surface-dark": "#111d27",
        "border-dark": "#223849",
        "surface-border": "#2c4458",
        "text-secondary": "#8fb0cc",
        "code-bg": "#0d1116",
        "success": "#22c55e",
        "danger": "#ef4444",
        "warning": "#f59e0b",
      },
      fontFamily: {
        "display": ["Space Grotesk", "sans-serif"],
        "mono": ["JetBrains Mono", "monospace"],
      },
      borderRadius: {
        "DEFAULT": "0.25rem",
        "lg": "0.5rem",
        "xl": "0.75rem",
        "full": "9999px"
      },
    },
  },
  plugins: [],
}
