import { defineConfig } from "vite";
import { fileURLToPath } from "node:url";
import path from "node:path";

const here = path.dirname(fileURLToPath(import.meta.url));

export default defineConfig({
  root: here,
  base: "/template_wasm/",
  server: {
    port: 5174,
    strictPort: false,
    fs: {
      allow: [here, path.resolve(here, ".."), path.resolve(here, "../..")]
    }
  },
  optimizeDeps: {
    exclude: ["@mujoco/mujoco"]
  },
  build: {
    target: "es2022"
  }
});
