import * as THREE from 'three';
import { CameraMode, VehicleState } from '../types';
import { targetVerticalFov } from './cameraProjection';

export class CameraController {
  public camera: THREE.PerspectiveCamera;
  public mode: CameraMode = 'chase';

  // Smoothed camera target and positions
  private currentPos: THREE.Vector3 = new THREE.Vector3(0, 4, -8);
  private currentLookAt: THREE.Vector3 = new THREE.Vector3(0, 1, 4);
  private currentFov: number = 52;

  // Orbit controls. Orbit is a momentary look-around view while left-dragging.
  private orbitAngle: number = Math.PI;
  private orbitPitch: number = 0.35;
  private orbitDistance: number = 7.5;
  private isMouseDown: boolean = false;
  private isDragging: boolean = false;
  private dragStartMouseX: number = 0;
  private dragStartMouseY: number = 0;
  private lastMouseX: number = 0;
  private lastMouseY: number = 0;
  private readonly dragThresholdPx: number = 3;

  // Latest vehicle pose lets a new orbit drag begin exactly from the camera's
  // current view rather than snapping to a hard-coded orbit angle/distance.
  private lastCarPos: THREE.Vector3 = new THREE.Vector3();
  private lastVehicleYaw: number = 0;
  private hasVehiclePose: boolean = false;

  constructor(camera: THREE.PerspectiveCamera) {
    this.camera = camera;
    this.currentFov = targetVerticalFov('chase', 0, camera.aspect);
    this.camera.fov = this.currentFov;
    this.camera.updateProjectionMatrix();
    this.setupMouseEvents();
  }

  private setupMouseEvents() {
    window.addEventListener('mousedown', (e) => {
      // Orbit look is intentionally desktop mouse-only and only starts from the
      // driving canvas, so HUD/UI clicks never steal the camera.
      if (e.button !== 0 || !(e.target instanceof HTMLCanvasElement)) return;

      this.isMouseDown = true;
      this.isDragging = false;
      this.dragStartMouseX = e.clientX;
      this.dragStartMouseY = e.clientY;
      this.lastMouseX = e.clientX;
      this.lastMouseY = e.clientY;
      this.syncOrbitToCurrentCamera();
    });

    window.addEventListener('mouseup', (e) => {
      if (e.button !== 0) return;
      this.endOrbitDrag();
    });

    window.addEventListener('blur', () => {
      this.endOrbitDrag();
    });

    window.addEventListener('mousemove', (e) => {
      if (!this.isMouseDown) return;

      const totalDx = e.clientX - this.dragStartMouseX;
      const totalDy = e.clientY - this.dragStartMouseY;
      if (!this.isDragging && Math.hypot(totalDx, totalDy) < this.dragThresholdPx) return;

      this.isDragging = true;

      const dx = e.clientX - this.lastMouseX;
      const dy = e.clientY - this.lastMouseY;
      this.lastMouseX = e.clientX;
      this.lastMouseY = e.clientY;

      this.orbitAngle -= dx * 0.006;
      this.orbitPitch = Math.max(0.05, Math.min(Math.PI / 2.2, this.orbitPitch + dy * 0.006));
    });

    window.addEventListener('wheel', (e) => {
      if (this.mode === 'orbit' || this.isDragging) {
        this.orbitDistance = Math.max(3.5, Math.min(22, this.orbitDistance + e.deltaY * 0.01));
      }
    });
  }

  private endOrbitDrag() {
    this.isMouseDown = false;
    this.isDragging = false;
  }

  private syncOrbitToCurrentCamera() {
    if (!this.hasVehiclePose) return;

    const orbitTarget = this.lastCarPos.clone().add(new THREE.Vector3(0, 0.7, 0));
    const offset = this.currentPos.clone().sub(orbitTarget);
    const radius = offset.length();
    if (radius < 0.001) return;

    this.orbitDistance = Math.max(3.5, Math.min(22, radius));

    const horizontalRadius = Math.hypot(offset.x, offset.z);
    const pitch = Math.atan2(offset.y, Math.max(0.001, horizontalRadius));
    this.orbitPitch = Math.max(0.05, Math.min(Math.PI / 2.2, pitch));

    // Store yaw relative to the vehicle so the look-around view follows the car
    // naturally through a turn while the driver keeps holding the mouse button.
    const worldOrbitAngle = Math.atan2(offset.x, offset.z);
    this.orbitAngle = worldOrbitAngle - this.lastVehicleYaw;
  }

  public setMode(mode: CameraMode) {
    this.mode = mode;
  }

  public nextMode(): CameraMode {
    // Orbit is no longer a permanent C-cycle camera. It is entered temporarily
    // by holding the left mouse button and dragging on the driving canvas.
    const modes: CameraMode[] = ['chase', 'close', 'hood', 'cockpit', 'drift'];
    const idx = modes.indexOf(this.mode);
    this.mode = modes[(idx + 1) % modes.length];
    return this.mode;
  }

  public update(dt: number, state: VehicleState) {
    const carPos = new THREE.Vector3(state.x, state.y, state.z);
    const speedKmh = state.speedKmh;
    const sinYaw = Math.sin(state.yaw);
    const cosYaw = Math.cos(state.yaw);

    this.lastCarPos.copy(carPos);
    this.lastVehicleYaw = state.yaw;
    this.hasVehiclePose = true;

    // Right-handed vehicle frame: +Z forward, +X left, so vehicle-right is -X at zero yaw.
    const forward = new THREE.Vector3(sinYaw, 0, cosYaw);
    const right = new THREE.Vector3(-cosYaw, 0, sinYaw);

    let targetPos = new THREE.Vector3();
    let targetLookAt = new THREE.Vector3();

    const activeMode: CameraMode = this.isDragging ? 'orbit' : this.mode;

    // Three.js PerspectiveCamera.fov is VERTICAL FOV. Author the driving cameras
    // in horizontal FOV and convert using the live viewport aspect ratio so a
    // metre has the same perspective scale on 16:9, ultrawide, and mobile.
    const targetFov = targetVerticalFov(activeMode, speedKmh, this.camera.aspect);

    switch (activeMode) {
      case 'chase': {
        // Position behind car with dynamic lookahead.
        const chaseDist = 6.8 + Math.min(2.0, speedKmh * 0.012);
        const chaseHeight = 2.4 - state.pitch * 1.5; // pitch compensation

        // Lookahead into turns
        const steerOffset = right.clone().multiplyScalar(state.actualSteerAngle * 1.2);
        targetPos = carPos
          .clone()
          .sub(forward.clone().multiplyScalar(chaseDist))
          .add(new THREE.Vector3(0, chaseHeight, 0))
          .add(steerOffset);

        targetLookAt = carPos
          .clone()
          .add(forward.clone().multiplyScalar(4.5))
          .add(new THREE.Vector3(0, 0.9, 0));
        break;
      }

      case 'close': {
        const chaseDist = 4.6;
        const chaseHeight = 1.45;

        targetPos = carPos
          .clone()
          .sub(forward.clone().multiplyScalar(chaseDist))
          .add(new THREE.Vector3(0, chaseHeight, 0));

        targetLookAt = carPos
          .clone()
          .add(forward.clone().multiplyScalar(3.0))
          .add(new THREE.Vector3(0, 0.6, 0));
        break;
      }

      case 'hood': {
        // Rigidly attached to hood
        targetPos = carPos
          .clone()
          .add(forward.clone().multiplyScalar(0.8))
          .add(new THREE.Vector3(0, 0.82 + state.heave, 0));

        targetLookAt = carPos
          .clone()
          .add(forward.clone().multiplyScalar(22))
          .add(new THREE.Vector3(0, 0.7, 0));
        break;
      }

      case 'cockpit': {
        // Driver's eye view inside cabin (-0.35 left, 0.95 high, 0.1 forward)
        const driverOffset = right.clone().multiplyScalar(-0.35).add(forward.clone().multiplyScalar(0.1));
        targetPos = carPos
          .clone()
          .add(driverOffset)
          .add(new THREE.Vector3(0, 0.98 + state.heave, 0));

        targetLookAt = carPos
          .clone()
          .add(driverOffset)
          .add(forward.clone().multiplyScalar(15))
          .add(new THREE.Vector3(0, 0.9, 0));
        break;
      }

      case 'drift': {
        // Wide high-angle cinematic camera highlighting tire smoke
        const slipDir = state.vx > 0 ? 1 : -1;
        const angleOffset = right.clone().multiplyScalar(slipDir * 3.8);
        targetPos = carPos
          .clone()
          .sub(forward.clone().multiplyScalar(7.5))
          .add(angleOffset)
          .add(new THREE.Vector3(0, 3.2, 0));

        targetLookAt = carPos.clone().add(new THREE.Vector3(0, 0.8, 0));
        break;
      }

      case 'orbit': {
        const radius = this.orbitDistance;
        const worldOrbitAngle = state.yaw + this.orbitAngle;
        const ox = Math.sin(worldOrbitAngle) * Math.cos(this.orbitPitch) * radius;
        const oy = Math.sin(this.orbitPitch) * radius;
        const oz = Math.cos(worldOrbitAngle) * Math.cos(this.orbitPitch) * radius;

        targetPos = carPos.clone().add(new THREE.Vector3(ox, Math.max(0.5, oy), oz));
        targetLookAt = carPos.clone().add(new THREE.Vector3(0, 0.7, 0));
        break;
      }
    }

    // Smooth camera lerp
    const lerpRate = activeMode === 'hood' || activeMode === 'cockpit' ? 0.45 : 0.12;
    this.currentPos.lerp(targetPos, Math.min(1.0, dt * (lerpRate * 60)));
    this.currentLookAt.lerp(targetLookAt, Math.min(1.0, dt * (lerpRate * 60)));

    this.currentFov += (targetFov - this.currentFov) * Math.min(1.0, dt * 8);
    this.camera.fov = this.currentFov;
    this.camera.updateProjectionMatrix();

    this.camera.position.copy(this.currentPos);

    // Explicitly maintain upright camera vector and apply subtle chassis roll compliance
    const rollAngle = (activeMode === 'chase' || activeMode === 'cockpit') ? -state.roll * 0.35 : 0;
    this.camera.up.set(-Math.sin(rollAngle), Math.cos(rollAngle), 0);
    this.camera.lookAt(this.currentLookAt);
  }
}
