import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";
export default defineConfig({
  base: "/inspect/",
  plugins: [vue()],
  build: { target: "es2020" },
  test: { include: ["src/*.test.js"] },
});
