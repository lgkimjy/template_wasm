import { mkdirSync, readdirSync, statSync } from "node:fs";
import { basename, extname, join, resolve } from "node:path";
import { spawnSync } from "node:child_process";

const projectRoot = resolve(new URL("..", import.meta.url).pathname);
const sourceMeshDir = resolve(
  projectRoot,
  "../MAHRU-WL_MPC_TOEJOINT/model/KIST MAHRU-WL/Meshes"
);
const outputDir = resolve(projectRoot, "public/assets/mahru_glb");

mkdirSync(outputDir, { recursive: true });

const stlFiles = readdirSync(sourceMeshDir)
  .filter((file) => extname(file).toLowerCase() === ".stl")
  .sort();

if (stlFiles.length === 0) {
  throw new Error(`No STL files found in ${sourceMeshDir}`);
}

for (const file of stlFiles) {
  const input = join(sourceMeshDir, file);
  const output = join(outputDir, `${basename(file, extname(file))}.glb`);
  const result = spawnSync("assimp", ["export", input, output, "-f", "glb2"], {
    encoding: "utf8"
  });

  if (result.status !== 0) {
    process.stderr.write(result.stdout || "");
    process.stderr.write(result.stderr || "");
    throw new Error(`assimp failed for ${file}`);
  }

  const srcSizeMb = statSync(input).size / 1024 / 1024;
  const outSizeMb = statSync(output).size / 1024 / 1024;
  console.log(`${file} -> ${basename(output)} (${srcSizeMb.toFixed(2)} MB -> ${outSizeMb.toFixed(2)} MB)`);
}
