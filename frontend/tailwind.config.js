/** @type {import('tailwindcss').Config} */
export default {
  darkMode: ["class", "class"],
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
  	extend: {
  		colors: {
  			'primary': '#1e94f6',
  			'primary-dark': '#167acc',
  			'background-light': '#f5f7f8',
  			'background-dark': '#0a0f14',
  			'panel-dark': '#121921',
  			'surface-dark': '#111d27',
  			'border-dark': '#223849',
  			'surface-border': '#2c4458',
  			'text-secondary': '#8fb0cc',
  			'code-bg': '#0d1116',
  			'success': '#22c55e',
  			'danger': '#ef4444',
  			'warning': '#f59e0b',
  			background: 'hsl(var(--background))',
  			foreground: 'hsl(var(--foreground))',
  			card: {
  				DEFAULT: 'hsl(var(--card))',
  				foreground: 'hsl(var(--card-foreground))'
  			},
  			popover: {
  				DEFAULT: 'hsl(var(--popover))',
  				foreground: 'hsl(var(--popover-foreground))'
  			},
  			primary: {
  				DEFAULT: 'hsl(var(--primary))',
  				foreground: 'hsl(var(--primary-foreground))'
  			},
  			secondary: {
  				DEFAULT: 'hsl(var(--secondary))',
  				foreground: 'hsl(var(--secondary-foreground))'
  			},
  			muted: {
  				DEFAULT: 'hsl(var(--muted))',
  				foreground: 'hsl(var(--muted-foreground))'
  			},
  			accent: {
  				DEFAULT: 'hsl(var(--accent))',
  				foreground: 'hsl(var(--accent-foreground))'
  			},
  			destructive: {
  				DEFAULT: 'hsl(var(--destructive))',
  				foreground: 'hsl(var(--destructive-foreground))'
  			},
  			border: 'hsl(var(--border))',
  			input: 'hsl(var(--input))',
  			ring: 'hsl(var(--ring))',
  			chart: {
  				'1': 'hsl(var(--chart-1))',
  				'2': 'hsl(var(--chart-2))',
  				'3': 'hsl(var(--chart-3))',
  				'4': 'hsl(var(--chart-4))',
  				'5': 'hsl(var(--chart-5))'
  			}
  		},
  		fontFamily: {
  			'display': [
  				'Space Grotesk',
  				'sans-serif'
  			],
  			'mono': [
  				'JetBrains Mono',
  				'monospace'
  			]
  		},
  		borderRadius: {
  			'DEFAULT': '0.25rem',
  			'lg': '0.5rem',
  			'xl': '0.75rem',
  			'full': '9999px',
  			lg: 'var(--radius)',
  			md: 'calc(var(--radius) - 2px)',
  			sm: 'calc(var(--radius) - 4px)'
  		}
  	}
  },
  plugins: [require("tailwindcss-animate")],
}
