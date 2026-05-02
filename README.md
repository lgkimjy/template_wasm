# MAHRU-WL WASM viewer starter

This is a browser-side MuJoCo/WebAssembly starter for MAHRU-WL. It is intentionally scoped to spawn, visualize, step, reset, and send a few direct actuator commands from the browser.

It is not a port of the full native `mahru_ctrl` WBC/MPC stack. That controller currently depends on native C++ libraries such as MuJoCo simulate/GLFW, HDF5, yaml-cpp, Eigen, OSQP/OsqpEigen, and eigen-quadprog. Porting that whole loop to WebAssembly is possible, but it should be handled as a separate controller-core extraction.

## Run

```sh
cd /Users/junyoungkim/hierlab_ws/MAHRU_WL/template_wasm
npm install
npm run assets:glb
npm run dev
```

Open the Vite URL printed in the terminal. In this session it is `http://127.0.0.1:5174`.

## What this app does

- Uses the official MuJoCo JavaScript/WebAssembly binding, `@mujoco/mujoco`.
- Imports `MAHRU-WL_MPC_TOEJOINT/model/KIST MAHRU-WL/KIST MAHRU-WL_w_Battery_passive.xml` as source MJCF so replay uses the same model family as the log.
- Converts the matching `model/KIST MAHRU-WL/Meshes/*.STL` files to `public/assets/mahru_glb/*.glb` with `assimp`.
- Uses a light MuJoCo runtime XML with mesh geoms stripped out, then renders the original visual meshes as a Three.js GLB overlay attached to MuJoCo body poses.
- Renders the MuJoCo `mjvScene` primitives, contact helpers, and world helpers with Three.js.
- Replays `public/trajectories/ssp_line_walk_onspot_qpos.csv`, copied from `MAHRU-WL_MPC_TOEJOINT/mimic/SSP_LINE_WALK_ONSPOT/mimic_data_with_timestamp.csv`.
- Exposes simple direct torque sliders for torso yaw and both wheel actuators.
- Provides toggles for contact points, MuJoCo contact-force visuals, a world-frame axis helper, and base-follow camera tracking.

## Notes

For real MAHRU control, keep browser code as UI/visualization and talk to a native safety-gated controller over WebSocket or ROS bridge. Browsers are not the right place to own hard realtime hardware torque loops.

The original STL model is heavy because the browser has to fetch the mesh assets, compile roughly 1.25 million mesh faces in MuJoCo WASM, and create Three.js geometry from the compiled mesh arrays. The GLB overlay path keeps the visible model detailed but avoids feeding those STL meshes into MuJoCo.

The current `assimp` GLBs are not decimated, so they preserve visual detail but are still large. A later pass with `gltfpack` or `meshoptimizer` would be the next step if this version feels visually correct but still too slow.

Replay mode is kinematic. It writes logged qpos frames into MuJoCo and calls `mj_forward()` for visualization; it does not run the native feedback controller.

Camera defaults are set in `CAMERA_VIEW` near the top of `src/main.js`. `azimuthDeg` and `elevationDeg` correspond to the usual viewer yaw/elevation idea, `distance` controls zoom, and `targetHeightOffset` keeps base tracking aimed slightly below the pelvis origin.
