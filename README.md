# MAHRU-WL WASM viewer starter

This is a browser-side MuJoCo/WebAssembly starter for MAHRU-WL. It is intentionally scoped to spawn, visualize, step, reset, and send a few direct actuator commands from the browser.

It is not a port of the full native `mahru_ctrl` WBC/MPC stack. That controller currently depends on native C++ libraries such as MuJoCo simulate/GLFW, HDF5, yaml-cpp, Eigen, OSQP/OsqpEigen, and eigen-quadprog. Porting that whole loop to WebAssembly is possible, but it should be handled as a separate controller-core extraction.

## Run

```sh
cd wasm_template
npm ci
npm run dev
```

Open the Vite URL printed in the terminal. Because this app is configured as a GitHub Pages project site, the local URL includes the repository base path, usually `http://127.0.0.1:5174/wasm_template/`.

## GitHub Pages

This repository is configured for GitHub Pages at:

```text
https://lgkimjy.github.io/wasm_template/
```

On GitHub, open `Settings -> Pages`, set `Build and deployment -> Source` to `GitHub Actions`, then push to `master`. The workflow in `.github/workflows/deploy.yml` runs `npm ci`, builds the Vite app, and deploys `dist/`.

## Project Webpage Embed

To embed this viewer in a project webpage, use the deployed GitHub Pages URL in an iframe:

```html
<center>
  <iframe
    src="https://lgkimjy.github.io/wasm_template/"
    style="width: 50%; height: 1000px; border: 0"
  ></iframe>
</center>
```

## What this app does

- Uses the official MuJoCo JavaScript/WebAssembly binding, `@mujoco/mujoco`.
- Imports `src/model/mahru_wl_battery_passive.xml` as source MJCF so the GitHub Pages build is self-contained.
- Converts the matching `model/KIST MAHRU-WL/Meshes/*.STL` files to `public/assets/mahru_glb/*.glb` with `assimp`.
- Uses a light MuJoCo runtime XML with mesh geoms stripped out, then renders the original visual meshes as a Three.js GLB overlay attached to MuJoCo body poses.
- Renders the MuJoCo `mjvScene` primitives, contact helpers, and world helpers with Three.js.
- Replays selectable qpos CSV rollouts from `public/trajectories/`, including the TOEJOINT on-spot walk and the `MAHRU-WL_control/mimic` dataset.
- Exposes simple direct torque sliders for torso yaw and both wheel actuators.
- Provides toggles for contact points, MuJoCo contact-force visuals, a world-frame axis helper, and base-follow camera tracking.

## Regenerate GLB Assets

The checked-in GLB files under `public/assets/mahru_glb/` are enough to run and deploy the viewer. Only run this if the source STL meshes change:

```sh
npm run assets:glb
```

That script expects `assimp` on `PATH` and the original MAHRU STL mesh directory at `../MAHRU-WL_MPC_TOEJOINT/model/KIST MAHRU-WL/Meshes`.

## Notes

For real MAHRU control, keep browser code as UI/visualization and talk to a native safety-gated controller over WebSocket or ROS bridge. Browsers are not the right place to own hard realtime hardware torque loops.

The original STL model is heavy because the browser has to fetch the mesh assets, compile roughly 1.25 million mesh faces in MuJoCo WASM, and create Three.js geometry from the compiled mesh arrays. The GLB overlay path keeps the visible model detailed but avoids feeding those STL meshes into MuJoCo.

The current `assimp` GLBs are not decimated, so they preserve visual detail but are still large. A later pass with `gltfpack` or `meshoptimizer` would be the next step if this version feels visually correct but still too slow.

Replay mode is kinematic. It writes logged qpos frames into MuJoCo and calls `mj_forward()` for visualization; it does not run the native feedback controller.

Camera defaults are set in `CAMERA_VIEW` near the top of `src/main.js`. `azimuthDeg` and `elevationDeg` correspond to the usual viewer yaw/elevation idea, `distance` controls zoom, and `targetHeightOffset` keeps base tracking aimed slightly below the pelvis origin.
