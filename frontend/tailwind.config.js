/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  darkMode: "class",
  theme: {
    extend: {
      colors: {
        "primary": "#1e94f6",
        "primary-dark": "#167acc",
        "background-light": "#f5f7f8",
        "background-dark": "#101a23",
        "panel-dark": "#16202a",
        "border-dark": "#223849",
        "text-secondary": "#8fb0cc",
        "code-bg": "#0d1116",
        "surface-dark": "#111d27",
        "surface-border": "#2c4458",
        "success": "#22c55e",
        "danger": "#ef4444",
        "warning": "#f59e0b",
      },
      fontFamily: {
        "display": ["Space Grotesk", "sans-serif"],
        "mono": ["JetBrains Mono", "monospace"],
      },
    },
  },
  plugins: [],
}
