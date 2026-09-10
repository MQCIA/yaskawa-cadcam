import type { Config } from "tailwindcss";

const config: Config = {
  content: [
    "./app/**/*.{js,ts,jsx,tsx,mdx}",
    "./components/**/*.{js,ts,jsx,tsx,mdx}",
  ],
  theme: {
    extend: {
      colors: {
        yaskawa: {
          blue: "#0033a0",
          accent: "#00a3e0",
        },
      },
    },
  },
  plugins: [],
};

export default config;
