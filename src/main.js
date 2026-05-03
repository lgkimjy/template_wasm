import "./styles.css";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import loadMujoco from "@mujoco/mujoco";
import mahruSourceXml from "./model/mahru_wl_battery_passive.xml?raw";

const APP_BASE = import.meta.env.BASE_URL || "/";
const GLB_ASSET_BASE = `${APP_BASE}assets/mahru_glb/`;
const TRAJECTORIES = [
  {
    id: "ssp_line_walk_onspot",
    label: "SSP_LINE_WALK_ONSPOT",
    url: `${APP_BASE}trajectories/ssp_line_walk_onspot_qpos.csv`,
    quaternionLayout: "xwyz"
  },
  {
    id: "mahru_wl_control_mimic",
    label: "MAHRU_WL_CONTROL_MIMIC",
    url: `${APP_BASE}trajectories/mahru_wl_control_mimic_qpos.csv`,
    quaternionLayout: "zwxy"
  }
];

const CAMERA_VIEW = {
  azimuthDeg: 47,
  elevationDeg: 16,
  distance: 4.0,
  targetHeightOffset: -0.14
};

const CTRL = {
  torsoYaw: 0,
  rWheel: 14,
  lWheel: 20
};

const els = {
  status: document.getElementById("status"),
  pause: document.getElementById("pause-button"),
  reset: document.getElementById("reset-button"),
  replay: document.getElementById("replay-button"),
  contacts: document.getElementById("contacts-button"),
  forces: document.getElementById("forces-button"),
  axes: document.getElementById("axes-button"),
  track: document.getElementById("track-button"),
  timeline: document.getElementById("timeline-input"),
  speed: document.getElementById("speed-input"),
  torso: document.getElementById("torso-input"),
  rWheel: document.getElementById("rwheel-input"),
  lWheel: document.getElementById("lwheel-input"),
  time: document.getElementById("sim-time"),
  trajectorySelect: document.getElementById("trajectory-select"),
  error: document.getElementById("error-message")
};

let mujoco;

class CapsuleGeometry extends THREE.BufferGeometry {
  constructor(radius = 1, length = 1, capSegments = 8, radialSegments = 16) {
    const path = new THREE.Path();
    path.absarc(0, -length / 2, radius, Math.PI * 1.5, 0, false);
    path.absarc(0, length / 2, radius, 0, Math.PI * 0.5, false);
    const lathe = new THREE.LatheGeometry(path.getPoints(capSegments), radialSegments);
    super();
    this.setIndex(lathe.getIndex());
    this.setAttribute("position", lathe.getAttribute("position"));
    this.setAttribute("normal", lathe.getAttribute("normal"));
    this.setAttribute("uv", lathe.getAttribute("uv"));
    lathe.dispose();
  }
}

function makeWorldAxes() {
  const group = new THREE.Group();
  const origin = new THREE.Vector3(0, 0, 0.035);
  const axes = [
    [new THREE.Vector3(1, 0, 0), 0xff5656],
    [new THREE.Vector3(0, 1, 0), 0x63d68a],
    [new THREE.Vector3(0, 0, 1), 0x5da8ff]
  ];
  for (const [direction, color] of axes) {
    const arrow = new THREE.ArrowHelper(direction, origin, 0.7, color, 0.09, 0.035);
    arrow.renderOrder = 10;
    group.add(arrow);
  }
  return group;
}

class MahruWasmApp {
  constructor() {
    this.model = null;
    this.data = null;
    this.sceneModel = null;
    this.option = new mujoco.MjvOption();
    this.perturb = new mujoco.MjvPerturb();
    this.cameraModel = new mujoco.MjvCamera();
    this.paused = false;
    this.mode = "replay";
    this.frame = null;
    this.meshes = [];
    this.geometryCache = new Map();
    this.visualManifest = null;
    this.visualMeshCache = new Map();
    this.visualParts = [];
    this.bodyAccessors = new Map();
    this.trajectory = null;
    this.trajectoryCache = new Map();
    this.selectedTrajectory = TRAJECTORIES[0];
    this.trajectoryRequest = 0;
    this.playbackFrame = 0;
    this.playbackTime = 0;
    this.lastWallTime = performance.now();
    this.baseTracking = true;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x111820);
    this.scene.fog = new THREE.Fog(0x111820, 7, 18);
    this.scene.add(new THREE.HemisphereLight(0xf4f8ff, 0x26323a, 1.8));
    this.worldAxes = makeWorldAxes();
    this.worldAxes.visible = false;
    this.scene.add(this.worldAxes);

    const keyLight = new THREE.DirectionalLight(0xffffff, 2.1);
    keyLight.position.set(-3.5, -4, 6);
    keyLight.castShadow = true;
    keyLight.shadow.mapSize.set(2048, 2048);
    keyLight.shadow.camera.near = 0.1;
    keyLight.shadow.camera.far = 12;
    keyLight.shadow.camera.left = -3;
    keyLight.shadow.camera.right = 3;
    keyLight.shadow.camera.top = 3;
    keyLight.shadow.camera.bottom = -3;
    this.scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0xaecfff, 0.65);
    fillLight.position.set(3, 2, 4);
    this.scene.add(fillLight);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    document.body.appendChild(this.renderer.domElement);

    this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.03, 80);
    this.camera.up.set(0, 0, 1);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.setCameraView(new THREE.Vector3(0, 0, 0.65));

    window.addEventListener("resize", () => this.resize());
  }

  async load() {
    this.setStatus("loading GLB");
    this.visualManifest = buildVisualManifest(mahruSourceXml);
    this.populateTrajectorySelect();
    const trajectoryTextPromise = fetchText(this.selectedTrajectory.url);
    const visualPromise = this.loadVisualOverlay();
    const vfs = new mujoco.MjVFS();
    try {
      this.setStatus("compiling light MJCF");
      this.model = mujoco.MjModel.from_xml_string(
        makeMahruXml(mahruSourceXml),
        vfs
      );
    } finally {
      vfs.delete();
    }
    this.data = new mujoco.MjData(this.model);
    this.sceneModel = new mujoco.MjvScene(this.model, 2 ** 15);
    mujoco.mj_forward(this.model, this.data);
    await visualPromise;
    this.bindVisualBodies();
    this.setStatus("loading rollout");
    this.trajectory = parseTrajectoryCsv(
      await trajectoryTextPromise,
      this.model.nq,
      this.selectedTrajectory
    );
    this.trajectoryCache.set(this.selectedTrajectory.id, this.trajectory);
    this.setupTrajectoryControls();
    if (this.trajectory) {
      this.mode = "replay";
      this.applyTrajectoryFrame(0);
    }
    this.updateModeUi();
  }

  populateTrajectorySelect() {
    els.trajectorySelect.replaceChildren();
    for (const trajectory of TRAJECTORIES) {
      const option = document.createElement("option");
      option.value = trajectory.id;
      option.textContent = trajectory.label;
      els.trajectorySelect.appendChild(option);
    }
    els.trajectorySelect.value = this.selectedTrajectory.id;
  }

  bindControls() {
    els.pause.addEventListener("click", () => {
      this.paused = !this.paused;
      this.updateModeUi();
    });

    els.reset.addEventListener("click", () => this.reset());
    window.addEventListener("keydown", (event) => {
      if (event.key !== "Backspace" || event.repeat) return;
      const target = event.target;
      const isTextEdit = target instanceof HTMLTextAreaElement
        || target?.isContentEditable
        || (target instanceof HTMLInputElement && target.type !== "range");
      if (isTextEdit) return;
      event.preventDefault();
      this.reset();
    });

    els.replay.addEventListener("click", () => {
      if (!this.trajectory) return;
      this.mode = this.mode === "replay" ? "sim" : "replay";
      this.lastWallTime = performance.now();
      if (this.mode === "replay") {
        this.applyTrajectoryFrame(this.playbackFrame);
      }
      this.updateModeUi();
    });

    els.contacts.addEventListener("click", () => {
      const flag = mujoco.mjtVisFlag.mjVIS_CONTACTPOINT.value;
      this.option.flags[flag] = !this.option.flags[flag];
      els.contacts.classList.toggle("active", this.option.flags[flag]);
    });

    els.forces.addEventListener("click", () => {
      const forceFlag = mujoco.mjtVisFlag.mjVIS_CONTACTFORCE.value;
      this.option.flags[forceFlag] = !this.option.flags[forceFlag];
      els.forces.classList.toggle("active", this.option.flags[forceFlag]);
    });

    els.axes.addEventListener("click", () => {
      this.worldAxes.visible = !this.worldAxes.visible;
      els.axes.classList.toggle("active", this.worldAxes.visible);
    });

    els.track.addEventListener("click", () => {
      this.baseTracking = !this.baseTracking;
      els.track.classList.toggle("active", this.baseTracking);
      if (this.baseTracking) this.updateBaseTracking(true);
    });

    els.timeline.addEventListener("input", () => {
      if (!this.trajectory) return;
      this.mode = "replay";
      this.paused = true;
      this.applyTrajectoryFrame(Number(els.timeline.value));
      this.updateModeUi();
    });

    els.trajectorySelect.addEventListener("change", () => {
      this.selectTrajectory(els.trajectorySelect.value);
    });

    for (const input of [els.torso, els.rWheel, els.lWheel]) {
      input.addEventListener("input", () => this.applyControls());
    }
    els.track.classList.toggle("active", this.baseTracking);
  }

  async loadVisualOverlay() {
    if (!this.visualManifest) return;
    const loader = new GLTFLoader();
    const uniqueFiles = Array.from(new Set(this.visualManifest.geoms.map((geom) => geom.file))).sort();

    await Promise.all(uniqueFiles.map(async (file) => {
      const gltf = await loadGltf(loader, glbUrlForStl(file));
      this.visualMeshCache.set(file, gltf.scene);
    }));

    for (const visualGeom of this.visualManifest.geoms) {
      const source = this.visualMeshCache.get(visualGeom.file);
      if (!source) continue;

      const object = source.clone(true);
      object.matrixAutoUpdate = false;
      object.visible = false;
      object.traverse((child) => {
        if (!child.isMesh) return;
        child.castShadow = true;
        child.receiveShadow = true;
        child.material = materialForRgba(visualGeom.rgba);
      });

      this.scene.add(object);
      this.visualParts.push({
        ...visualGeom,
        object,
        bodyAccessor: null,
        bodyMatrix: new THREE.Matrix4()
      });
    }
  }

  bindVisualBodies() {
    for (const part of this.visualParts) {
      if (!this.bodyAccessors.has(part.bodyName)) {
        this.bodyAccessors.set(part.bodyName, this.data.body(part.bodyName));
      }
      part.bodyAccessor = this.bodyAccessors.get(part.bodyName);
    }
  }

  async selectTrajectory(trajectoryId) {
    const nextTrajectory = TRAJECTORIES.find((trajectory) => trajectory.id === trajectoryId);
    if (!nextTrajectory || nextTrajectory.id === this.selectedTrajectory.id || !this.model) return;

    this.selectedTrajectory = nextTrajectory;
    els.trajectorySelect.value = nextTrajectory.id;
    els.trajectorySelect.disabled = true;
    this.setStatus("loading rollout");
    const requestId = ++this.trajectoryRequest;

    try {
      let trajectory = this.trajectoryCache.get(nextTrajectory.id);
      if (!trajectory) {
        trajectory = parseTrajectoryCsv(await fetchText(nextTrajectory.url), this.model.nq, nextTrajectory);
        this.trajectoryCache.set(nextTrajectory.id, trajectory);
      }
      if (requestId !== this.trajectoryRequest) return;

      this.trajectory = trajectory;
      this.mode = "replay";
      this.paused = false;
      this.playbackFrame = 0;
      this.playbackTime = trajectory.times[0];
      this.setupTrajectoryControls();
      this.applyTrajectoryFrame(0);
      this.updateBaseTracking(true);
      this.updateModeUi();
    } catch (error) {
      console.error(error);
      this.setStatus("error", error);
    } finally {
      if (requestId === this.trajectoryRequest) {
        els.trajectorySelect.disabled = false;
      }
    }
  }

  reset() {
    if (!this.model || !this.data) return;
    if (this.mode === "replay" && this.trajectory) {
      this.playbackTime = this.trajectory.times[0];
      this.applyTrajectoryFrame(0);
      this.updateModeUi();
      return;
    }

    mujoco.mj_resetData(this.model, this.data);
    mujoco.mj_forward(this.model, this.data);
    for (const input of [els.torso, els.rWheel, els.lWheel]) {
      input.value = "0";
    }
    this.applyControls();
  }

  applyControls() {
    if (this.mode === "replay") return;
    if (!this.data?.ctrl) return;
    this.data.ctrl[CTRL.torsoYaw] = Number(els.torso.value);
    this.data.ctrl[CTRL.rWheel] = Number(els.rWheel.value);
    this.data.ctrl[CTRL.lWheel] = Number(els.lWheel.value);
  }

  update() {
    if (!this.model || !this.data) return;

    const now = performance.now();
    const wallDt = Math.min(0.1, Math.max(0, (now - this.lastWallTime) / 1000));
    this.lastWallTime = now;

    this.controls.update();

    if (this.mode === "replay" && this.trajectory) {
      if (!this.paused) {
        this.advanceReplay(wallDt);
      } else {
        this.applyTrajectoryFrame(this.playbackFrame);
      }
    } else if (!this.paused) {
      this.applyControls();
      const start = this.data.time;
      while (this.data.time - start < 1 / 60) {
        mujoco.mj_step(this.model, this.data);
      }
    }

    this.updateBaseTracking();
    mujoco.mjv_updateScene(
      this.model,
      this.data,
      this.option,
      this.perturb,
      this.cameraModel,
      mujoco.mjtCatBit.mjCAT_ALL.value,
      this.sceneModel
    );

    this.syncThreeScene();
    this.syncVisualOverlay();
    this.updateReadout();
  }

  advanceReplay(wallDt) {
    const times = this.trajectory.times;
    const endTime = times[times.length - 1];
    const startTime = times[0];
    const duration = endTime - startTime;
    const speed = Number(els.speed.value) || 1;

    this.playbackTime += wallDt * speed;
    if (duration > 0) {
      while (this.playbackTime > endTime) {
        this.playbackTime = startTime + (this.playbackTime - endTime);
      }
    }
    this.applyTrajectoryFrame(this.frameForTime(this.playbackTime));
  }

  frameForTime(time) {
    const times = this.trajectory.times;
    let lo = 0;
    let hi = times.length - 1;
    while (lo < hi) {
      const mid = Math.floor((lo + hi) / 2);
      if (times[mid] < time) lo = mid + 1;
      else hi = mid;
    }
    if (lo > 0 && Math.abs(times[lo - 1] - time) < Math.abs(times[lo] - time)) {
      return lo - 1;
    }
    return lo;
  }

  baseTarget() {
    const qpos = this.data?.qpos;
    if (!qpos) return new THREE.Vector3(0, 0, 0.65);
    return new THREE.Vector3(qpos[0], qpos[1], qpos[2] + CAMERA_VIEW.targetHeightOffset);
  }

  setCameraView(target) {
    const azimuth = THREE.MathUtils.degToRad(CAMERA_VIEW.azimuthDeg);
    const elevation = THREE.MathUtils.degToRad(CAMERA_VIEW.elevationDeg);
    const horizontal = CAMERA_VIEW.distance * Math.cos(elevation);
    const offset = new THREE.Vector3(
      horizontal * Math.cos(azimuth),
      horizontal * Math.sin(azimuth),
      CAMERA_VIEW.distance * Math.sin(elevation)
    );
    this.controls.target.copy(target);
    this.camera.position.copy(target).add(offset);
    this.controls.update();
  }

  updateBaseTracking(force = false) {
    if (!this.baseTracking || !this.data) return;
    const target = this.baseTarget();
    const delta = target.sub(this.controls.target);
    if (!force && delta.lengthSq() < 1e-10) return;
    this.controls.target.add(delta);
    this.camera.position.add(delta);
  }

  setupTrajectoryControls() {
    if (!this.trajectory) {
      els.timeline.disabled = true;
      els.speed.disabled = true;
      els.replay.disabled = true;
      return;
    }

    const lastFrame = this.trajectory.qpos.length - 1;
    els.timeline.min = "0";
    els.timeline.max = String(lastFrame);
    els.timeline.value = "0";
    els.timeline.disabled = false;
    els.speed.disabled = false;
    els.replay.disabled = false;
  }

  applyTrajectoryFrame(frameIndex) {
    if (!this.trajectory || !this.data) return;

    const lastFrame = this.trajectory.qpos.length - 1;
    const frame = Math.max(0, Math.min(lastFrame, Math.round(frameIndex)));
    const qpos = this.trajectory.qpos[frame];
    for (let i = 0; i < this.model.nq; i += 1) {
      this.data.qpos[i] = qpos[i];
    }
    normalizeQuaternion(this.data.qpos, 3);
    if (typeof this.data.qvel?.fill === "function") {
      this.data.qvel.fill(0);
    }
    this.data.time = this.trajectory.times[frame];
    this.playbackFrame = frame;
    this.playbackTime = this.trajectory.times[frame];
    els.timeline.value = String(frame);
    mujoco.mj_forward(this.model, this.data);
  }

  updateModeUi() {
    const replaying = this.mode === "replay" && this.trajectory;
    els.pause.textContent = this.paused ? "Resume" : "Pause";
    els.replay.classList.toggle("active", replaying);
    els.timeline.disabled = !this.trajectory;
    els.speed.disabled = !this.trajectory;
    for (const input of [els.torso, els.rWheel, els.lWheel]) {
      input.disabled = replaying;
    }
    if (this.model) {
      this.setStatus(replaying ? "replay" : "sim");
    }
  }

  syncVisualOverlay() {
    for (const part of this.visualParts) {
      const body = part.bodyAccessor;
      if (!body) continue;
      setMatrixFromMujocoXmat(part.bodyMatrix, body.xpos, body.xmat);
      part.object.matrix.multiplyMatrices(part.bodyMatrix, part.localMatrix);
      part.object.matrixWorldNeedsUpdate = true;
      part.object.visible = true;
    }
  }

  syncThreeScene() {
    const geoms = this.sceneModel.geoms;
    const geomCount = geoms.size();
    for (let i = 0; i < geomCount; i += 1) {
      const mjvGeom = geoms.get(i);
      let mesh = this.meshes[i];
      if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_MESH.value) {
        if (mesh) mesh.visible = false;
        mjvGeom.delete();
        continue;
      }
      const geometryKey = this.geometryKey(mjvGeom);

      if (!mesh) {
        const geometry = this.geometryFor(mjvGeom);
        const material = new THREE.MeshPhongMaterial({
          color: new THREE.Color(mjvGeom.rgba[0], mjvGeom.rgba[1], mjvGeom.rgba[2]),
          opacity: mjvGeom.rgba[3],
          transparent: mjvGeom.rgba[3] < 1
        });
        mesh = new THREE.Mesh(geometry, material);
        mesh.userData.geometryKey = geometryKey;
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.matrixAutoUpdate = false;
        this.meshes[i] = mesh;
        this.scene.add(mesh);
      } else if (mesh.userData.geometryKey !== geometryKey) {
        mesh.geometry = this.geometryFor(mjvGeom);
        mesh.userData.geometryKey = geometryKey;
      }

      mesh.material.color.setRGB(mjvGeom.rgba[0], mjvGeom.rgba[1], mjvGeom.rgba[2]);
      mesh.material.opacity = mjvGeom.rgba[3];
      mesh.material.transparent = mjvGeom.rgba[3] < 1;

      mesh.matrix.set(
        mjvGeom.mat[0], mjvGeom.mat[1], mjvGeom.mat[2], mjvGeom.pos[0],
        mjvGeom.mat[3], mjvGeom.mat[4], mjvGeom.mat[5], mjvGeom.pos[1],
        mjvGeom.mat[6], mjvGeom.mat[7], mjvGeom.mat[8], mjvGeom.pos[2],
        0, 0, 0, 1
      );
      mesh.matrixWorldNeedsUpdate = true;
      mesh.visible = true;
      mjvGeom.delete();
    }
    geoms.delete();

    for (let i = geomCount; i < this.meshes.length; i += 1) {
      const mesh = this.meshes[i];
      if (mesh) mesh.visible = false;
    }
  }

  geometryKey(mjvGeom) {
    if (isDynamicVisualGeom(mjvGeom.type)) {
      return JSON.stringify([
        mjvGeom.type,
        ...Array.from(mjvGeom.size).map((value) => Math.round(value * 1000) / 1000)
      ]);
    }
    return JSON.stringify([mjvGeom.type, Array.from(mjvGeom.size), mjvGeom.dataid]);
  }

  geometryFor(mjvGeom) {
    const key = this.geometryKey(mjvGeom);
    const cached = this.geometryCache.get(key);
    if (cached) return cached;

    let geometry;
    if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_PLANE.value) {
      const sx = mjvGeom.size[0] || 8;
      const sy = mjvGeom.size[1] || 8;
      geometry = new THREE.PlaneGeometry(2 * sx, 2 * sy);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_SPHERE.value) {
      geometry = new THREE.SphereGeometry(mjvGeom.size[0], 32, 18);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
      geometry = new CapsuleGeometry(mjvGeom.size[0], 2 * mjvGeom.size[2], 8, 24);
      geometry.rotateX(Math.PI / 2);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
      geometry = new THREE.CylinderGeometry(mjvGeom.size[0], mjvGeom.size[0], 2 * mjvGeom.size[2], 32);
      geometry.rotateX(Math.PI / 2);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
      geometry = new THREE.SphereGeometry(1, 32, 18);
      geometry.scale(mjvGeom.size[0], mjvGeom.size[1], mjvGeom.size[2]);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_BOX.value) {
      geometry = new THREE.BoxGeometry(2 * mjvGeom.size[0], 2 * mjvGeom.size[1], 2 * mjvGeom.size[2]);
    } else if (isArrowGeom(mjvGeom.type)) {
      geometry = this.arrowGeometryFor(mjvGeom);
    } else if (mjvGeom.type === mujoco.mjtGeom.mjGEOM_LINE.value) {
      geometry = this.arrowGeometryFor(mjvGeom);
    } else {
      geometry = new THREE.BoxGeometry(0.03, 0.03, 0.03);
    }

    this.geometryCache.set(key, geometry);
    return geometry;
  }

  arrowGeometryFor(mjvGeom) {
    const radius = Math.max(0.004, mjvGeom.size[0] || 0.008);
    const length = Math.max(0.025, mjvGeom.size[2] || 0.08);
    const headLength = Math.min(length * 0.38, Math.max(radius * 6, 0.035));
    const doubleHeaded = mjvGeom.type === mujoco.mjtGeom.mjGEOM_ARROW2.value;
    const shaftStart = doubleHeaded ? headLength : 0;
    const shaftEnd = Math.max(shaftStart, length - headLength);
    const shaftLength = Math.max(shaftEnd - shaftStart, length * 0.2);
    const headRadius = Math.max(radius * 3.2, 0.016);
    const parts = [];

    const shaft = new THREE.CylinderGeometry(radius, radius, shaftLength, 12);
    shaft.rotateX(Math.PI / 2);
    shaft.translate(0, 0, shaftStart + shaftLength / 2);
    parts.push(shaft);

    const head = new THREE.ConeGeometry(headRadius, headLength, 18);
    head.rotateX(Math.PI / 2);
    head.translate(0, 0, length - headLength / 2);
    parts.push(head);

    if (doubleHeaded) {
      const tail = new THREE.ConeGeometry(headRadius, headLength, 18);
      tail.rotateX(-Math.PI / 2);
      tail.translate(0, 0, headLength / 2);
      parts.push(tail);
    }

    const geometry = mergeGeometries(parts, false);
    for (const part of parts) part.dispose();
    return geometry;
  }

  updateReadout() {
    els.time.textContent = this.mode === "replay" && this.trajectory
      ? `log ${this.data.time.toFixed(3)} sec · frame ${this.playbackFrame}`
      : `t ${this.data.time.toFixed(3)}`;
  }

  resize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  run() {
    const tick = () => {
      try {
        this.update();
        this.renderer.render(this.scene, this.camera);
      } catch (error) {
        console.error(error);
        this.setStatus("error", error);
      }
      this.frame = requestAnimationFrame(tick);
    };
    tick();
  }

  dispose() {
    if (this.frame) cancelAnimationFrame(this.frame);
    this.sceneModel?.delete();
    this.cameraModel?.delete();
    this.perturb?.delete();
    this.option?.delete();
    for (const accessor of this.bodyAccessors.values()) {
      accessor?.delete?.();
    }
    this.data?.delete();
    this.model?.delete();
    this.controls.dispose();
    this.renderer.dispose();
  }

  setStatus(value, error = null) {
    els.status.textContent = value;
    els.error.textContent = error ? String(error.message || error) : "";
  }
}

function loadGltf(loader, url) {
  return new Promise((resolve, reject) => {
    loader.load(url, resolve, undefined, reject);
  });
}

function glbUrlForStl(file) {
  return `${GLB_ASSET_BASE}${file.replace(/\.stl$/i, ".glb")}`;
}

function materialForRgba(rgba) {
  return new THREE.MeshPhongMaterial({
    color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
    opacity: rgba[3],
    transparent: rgba[3] < 1,
    shininess: 55,
    specular: new THREE.Color(0x20242a)
  });
}

function buildVisualManifest(source) {
  const doc = parseXml(source);
  const meshAssets = new Map();
  for (const mesh of doc.querySelectorAll("asset > mesh")) {
    const name = mesh.getAttribute("name");
    const file = mesh.getAttribute("file");
    if (!name || !file) continue;
    meshAssets.set(name, {
      file,
      scale: parseVector(mesh.getAttribute("scale"), [1, 1, 1])
    });
  }

  const geoms = [];
  for (const geom of doc.querySelectorAll("worldbody geom[mesh]")) {
    const meshName = geom.getAttribute("mesh");
    const meshAsset = meshAssets.get(meshName);
    const body = geom.closest("body");
    if (!meshAsset || !body) continue;
    geoms.push({
      bodyName: body.getAttribute("name"),
      meshName,
      file: meshAsset.file,
      rgba: parseVector(geom.getAttribute("rgba"), [0.65, 0.67, 0.68, 1]),
      localMatrix: localMatrixForGeom(geom, meshAsset.scale)
    });
  }

  return { geoms };
}

function localMatrixForGeom(geom, assetScale) {
  const position = parseVector(geom.getAttribute("pos"), [0, 0, 0]);
  const scale = new THREE.Vector3(assetScale[0], assetScale[1], assetScale[2]);
  const quaternion = orientationForElement(geom);
  const matrix = new THREE.Matrix4();
  matrix.compose(new THREE.Vector3(position[0], position[1], position[2]), quaternion, scale);
  return matrix;
}

function orientationForElement(element) {
  const quat = parseOptionalVector(element.getAttribute("quat"));
  if (quat) return new THREE.Quaternion(quat[1], quat[2], quat[3], quat[0]).normalize();

  const axisAngle = parseOptionalVector(element.getAttribute("axisangle"));
  if (axisAngle) {
    const axis = new THREE.Vector3(axisAngle[0], axisAngle[1], axisAngle[2]).normalize();
    return new THREE.Quaternion().setFromAxisAngle(axis, axisAngle[3]);
  }

  const euler = parseOptionalVector(element.getAttribute("euler"));
  if (euler) {
    return new THREE.Quaternion().setFromEuler(new THREE.Euler(euler[0], euler[1], euler[2], "XYZ"));
  }

  return new THREE.Quaternion();
}

function setMatrixFromMujocoXmat(matrix, xpos, xmat) {
  matrix.set(
    xmat[0], xmat[1], xmat[2], xpos[0],
    xmat[3], xmat[4], xmat[5], xpos[1],
    xmat[6], xmat[7], xmat[8], xpos[2],
    0, 0, 0, 1
  );
}

async function fetchText(url) {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Failed to load ${url}: ${response.status}`);
  }
  return response.text();
}

function parseTrajectoryCsv(text, nq, config = {}) {
  const quaternionLayout = config.quaternionLayout || "xwyz";
  const times = [];
  const qpos = [];
  const lines = text.split(/\r?\n/);

  for (const line of lines) {
    const trimmed = line.trim();
    if (!trimmed) continue;

    const values = trimmed.split(",").map(Number);
    if (values.some((value) => !Number.isFinite(value))) continue;

    const hasTime = values.length >= nq + 1 || (nq === 28 && values.length === 27);
    const start = hasTime ? 1 : 0;
    const sourceLength = values.length - start;

    let frame = null;
    if (sourceLength >= nq) {
      frame = Float64Array.from(values.slice(start, start + nq));
    } else if (nq === 28 && sourceLength >= 26) {
      frame = expandMahruToeJointQpos(values.slice(start, start + 26));
    }
    if (!frame) continue;

    convertLoggedBaseQuatToMujoco(frame, quaternionLayout);
    times.push(hasTime ? values[0] : qpos.length * 0.001);
    qpos.push(frame);
  }

  if (qpos.length === 0) {
    throw new Error(`No ${nq}-DoF qpos frames found in rollout CSV`);
  }

  return { times, qpos };
}

function expandMahruToeJointQpos(source) {
  const qpos = new Float64Array(28);
  qpos.set(source.slice(0, 20), 0);
  qpos[20] = 0;
  qpos[21] = source[20];
  qpos.set(source.slice(21, 25), 22);
  qpos[26] = 0;
  qpos[27] = source[25];
  return qpos;
}

function convertLoggedBaseQuatToMujoco(qpos, layout) {
  const logged0 = qpos[3];
  const logged1 = qpos[4];
  const logged2 = qpos[5];
  const logged3 = qpos[6];

  if (layout === "zwxy") {
    // MAHRU-WL_control logs MuJoCo [w, x, y, z] as [z, w, x, y].
    qpos[3] = logged1;
    qpos[4] = logged2;
    qpos[5] = logged3;
    qpos[6] = logged0;
  } else {
    qpos[3] = logged1;
    qpos[4] = logged0;
    qpos[5] = logged2;
    qpos[6] = logged3;
  }
  normalizeQuaternion(qpos, 3);
}

function isArrowGeom(type) {
  return type === mujoco.mjtGeom.mjGEOM_ARROW.value
    || type === mujoco.mjtGeom.mjGEOM_ARROW1.value
    || type === mujoco.mjtGeom.mjGEOM_ARROW2.value;
}

function isDynamicVisualGeom(type) {
  return isArrowGeom(type) || type === mujoco.mjtGeom.mjGEOM_LINE.value;
}

function parseXml(source) {
  const parser = new DOMParser();
  const doc = parser.parseFromString(source, "application/xml");
  const parserError = doc.querySelector("parsererror");
  if (parserError) {
    throw new Error(parserError.textContent || "XML parse error");
  }
  return doc;
}

function parseOptionalVector(text) {
  if (!text) return null;
  const values = text.trim().split(/\s+/).map(Number);
  return values.every(Number.isFinite) ? values : null;
}

function parseVector(text, fallback) {
  const values = parseOptionalVector(text);
  if (!values) return fallback.slice();
  const parsed = fallback.slice();
  for (let i = 0; i < Math.min(values.length, parsed.length); i += 1) {
    parsed[i] = values[i];
  }
  return parsed;
}

function stripMeshVisuals(doc) {
  for (const geom of doc.querySelectorAll("worldbody geom[mesh]")) {
    const name = geom.getAttribute("name");
    if (name === "RWheel" || name === "LWheel") {
      geom.removeAttribute("mesh");
      geom.setAttribute("type", "ellipsoid");
      geom.setAttribute("size", "0.075 0.075 0.015");
      geom.setAttribute("rgba", "0 0 0 0");
      continue;
    }
    geom.remove();
  }
  for (const mesh of doc.querySelectorAll("asset > mesh")) {
    mesh.remove();
  }
  for (const texture of doc.querySelectorAll("asset > texture")) {
    texture.remove();
  }
  for (const material of doc.querySelectorAll("asset > material")) {
    material.removeAttribute("texture");
    material.removeAttribute("texrepeat");
    material.removeAttribute("texuniform");
  }
  const compiler = doc.querySelector("compiler");
  compiler?.removeAttribute("meshdir");
  compiler?.removeAttribute("texturedir");
}

function makeMahruXml(source) {
  const doc = parseXml(source);
  stripMeshVisuals(doc);

  let worldbody = doc.querySelector("worldbody");
  if (!worldbody) {
    worldbody = doc.createElement("worldbody");
    doc.documentElement.appendChild(worldbody);
  }

  let ground = worldbody.querySelector('geom[name="Ground"]');
  if (!ground) {
    ground = doc.createElement("geom");
    ground.setAttribute("name", "Ground");
    ground.setAttribute("type", "plane");
    ground.setAttribute("pos", "0 0 0");
    worldbody.insertBefore(ground, worldbody.firstChild);
  }
  ground.removeAttribute("material");
  ground.setAttribute("size", "6 6 0.05");
  ground.setAttribute("rgba", "0.18 0.22 0.25 1");
  ground.setAttribute("friction", "1 0.1 0.1");
  ground.setAttribute("condim", "3");

  for (const [pos, dir] of [
    ["3 -4 5", "-0.4 0.5 -1"],
    ["-3 3 5", "0.4 -0.4 -1"]
  ]) {
    const light = doc.createElement("light");
    light.setAttribute("directional", "true");
    light.setAttribute("pos", pos);
    light.setAttribute("dir", dir);
    light.setAttribute("diffuse", "0.45 0.45 0.45");
    light.setAttribute("specular", "0.1 0.1 0.1");
    light.setAttribute("castshadow", "false");
    worldbody.insertBefore(light, worldbody.firstChild);
  }

  return new XMLSerializer().serializeToString(doc);
}

function normalizeQuaternion(values, offset) {
  const norm = Math.hypot(values[offset], values[offset + 1], values[offset + 2], values[offset + 3]);
  if (norm <= 1e-12) return;
  values[offset] /= norm;
  values[offset + 1] /= norm;
  values[offset + 2] /= norm;
  values[offset + 3] /= norm;
}

async function main() {
  let app;
  try {
    mujoco = await loadMujoco();
    app = new MahruWasmApp();
    await app.load();
    app.bindControls();
    window.addEventListener("unload", () => app.dispose());
    app.run();
  } catch (error) {
    console.error(error);
    els.status.textContent = "error";
    els.error.textContent = String(error.message || error);
  }
}

main();
