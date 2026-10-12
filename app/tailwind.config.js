/** @type {import('tailwindcss').Config} */
export default {
  content: ["./index.html", "./src/**/*.{ts,tsx}"],
  theme: {
    extend: {
      colors: {
        hud: { bg: "rgba(5,10,15,0.62)", fg: "#f2f5f7", dim: "#a9b4bd", ok: "#3ddc84", warn: "#ffb020", bad: "#ff4d4f" },
      },
      minHeight: { touch: "44px" },
      minWidth: { touch: "44px" },
    },
  },
  plugins: [],
};
