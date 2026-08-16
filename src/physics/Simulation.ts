import { VehicleConfig, VehicleState, ControlInputs } from '../types';
import { Vehicle } from './Vehicle';
import { ISurfaceProvider } from './SurfaceProvider';
import { PhysicsMath } from './math/PhysicsMath';

export class Simulation {
  public vehicle: Vehicle;
  public fixedDt: number = 1.0 / 120.0;
  public maxSubSteps: number = 8;
  public accumulatedTime: number = 0;
  public totalSimTime: number = 0;
  public stepCount: number = 0;

  private previousState: VehicleState;
  private currentState: VehicleState;

  constructor(config: VehicleConfig, surfaceProvider?: ISurfaceProvider) {
    this.vehicle = new Vehicle(config, surfaceProvider);
    this.previousState = this.vehicle.getState();
    this.currentState = this.vehicle.getState();
  }

  public reset(x: number = 0, z: number = 0, yaw: number = 0) {
    this.vehicle.reset(x, z, yaw);
    this.accumulatedTime = 0;
    this.totalSimTime = 0;
    this.stepCount = 0;
    this.previousState = this.vehicle.getState();
    this.currentState = this.vehicle.getState();
  }

  public setConfig(newConfig: VehicleConfig) {
    const oldCgHeight = this.vehicle.config.centerOfGravityHeight;
    this.vehicle.setConfig(newConfig);

    // Vehicle.setConfig updates the subsystem tuning, but the rigid-body mass and
    // principal inertias also have to change. Without this, selecting a 2050 kg
    // preset still physically simulated the original 1540 kg chassis.
    const m = newConfig.mass;
    const L = newConfig.wheelbase;
    const W = newConfig.trackWidth;
    const H = newConfig.centerOfGravityHeight;

    const geometricPitch = (m / 12) * (L * L + H * H) * 1.5;
    const geometricYaw = (m / 12) * (L * L + W * W) * 1.1;
    const geometricRoll = (m / 12) * (W * W + H * H) * 1.6;

    const configuredPitch = Number((newConfig as any).chassisPitchInertia);
    const configuredRoll = Number((newConfig as any).chassisRollInertia);

    this.vehicle.rigidBody.config = {
      mass: Math.max(1, m),
      inertia: PhysicsMath.vec3(
        Number.isFinite(configuredPitch) && configuredPitch > 0 ? configuredPitch : geometricPitch,
        geometricYaw,
        Number.isFinite(configuredRoll) && configuredRoll > 0 ? configuredRoll : geometricRoll
      ),
      centerOfGravityHeight: H,
    };

    // Preserve the current ground-relative ride height when CG height is changed
    // from the tuning UI or a preset swap.
    if (Number.isFinite(oldCgHeight) && Number.isFinite(H)) {
      this.vehicle.rigidBody.position.y += H - oldCgHeight;
    }

    this.currentState = this.vehicle.getState();
    this.previousState = this.currentState;
  }

  /**
   * Advance simulation by variable render frame deltaTime.
   * Uses fixed 120 Hz accumulator with state interpolation.
   */
  public advance(deltaTime: number, inputs: ControlInputs): VehicleState {
    const clampedDelta = Math.min(deltaTime, 0.1);
    this.accumulatedTime += clampedDelta;

    let subStepsTaken = 0;

    while (this.accumulatedTime >= this.fixedDt && subStepsTaken < this.maxSubSteps) {
      this.previousState = this.currentState;
      this.vehicle.step(inputs, this.fixedDt);
      this.currentState = this.vehicle.getState();

      this.accumulatedTime -= this.fixedDt;
      this.totalSimTime += this.fixedDt;
      this.stepCount++;
      subStepsTaken++;
    }

    if (this.accumulatedTime > this.fixedDt * 2) {
      this.accumulatedTime = 0;
    }

    const alpha = Math.min(1.0, Math.max(0, this.accumulatedTime / this.fixedDt));
    return this.interpolateState(this.previousState, this.currentState, alpha);
  }

  public stepExplicit(inputs: ControlInputs, steps: number = 1): VehicleState {
    for (let i = 0; i < steps; i++) {
      this.vehicle.step(inputs, this.fixedDt);
      this.totalSimTime += this.fixedDt;
      this.stepCount++;
    }
    this.currentState = this.vehicle.getState();
    this.previousState = this.currentState;
    return this.currentState;
  }

  private interpolateState(prev: VehicleState, curr: VehicleState, alpha: number): VehicleState {
    if (alpha <= 0.001) return prev;
    if (alpha >= 0.999) return curr;

    const lerp = PhysicsMath.lerp;
    const yawDiff = ((((curr.yaw - prev.yaw) % (Math.PI * 2)) + Math.PI * 3) % (Math.PI * 2)) - Math.PI;

    return {
      ...curr,
      x: lerp(prev.x, curr.x, alpha),
      y: lerp(prev.y, curr.y, alpha),
      z: lerp(prev.z, curr.z, alpha),
      yaw: prev.yaw + yawDiff * alpha,
      pitch: lerp(prev.pitch, curr.pitch, alpha),
      roll: lerp(prev.roll, curr.roll, alpha),
      speedMs: lerp(prev.speedMs, curr.speedMs, alpha),
      speedKmh: lerp(prev.speedKmh, curr.speedKmh, alpha),
      speedMph: lerp(prev.speedMph, curr.speedMph, alpha),
      rpm: lerp(prev.rpm, curr.rpm, alpha),
      lateralG: lerp(prev.lateralG, curr.lateralG, alpha),
      longitudinalG: lerp(prev.longitudinalG, curr.longitudinalG, alpha),
      verticalG: lerp(prev.verticalG, curr.verticalG, alpha),
      actualSteerAngle: lerp(prev.actualSteerAngle, curr.actualSteerAngle, alpha),
      turboBoostPsi: lerp(prev.turboBoostPsi, curr.turboBoostPsi, alpha),
      wheels: curr.wheels.map((w, i) => {
        const pw = prev.wheels[i];
        return {
          ...w,
          suspensionCompression: lerp(pw.suspensionCompression, w.suspensionCompression, alpha),
          verticalTravelM: lerp(pw.verticalTravelM, w.verticalTravelM, alpha),
          rotationAngle: lerp(pw.rotationAngle, w.rotationAngle, alpha),
          steerAngle: lerp(pw.steerAngle, w.steerAngle, alpha),
          forceVectorLong: lerp(pw.forceVectorLong, w.forceVectorLong, alpha),
          forceVectorLat: lerp(pw.forceVectorLat, w.forceVectorLat, alpha),
          forceVectorNorm: lerp(pw.forceVectorNorm, w.forceVectorNorm, alpha),
        };
      }) as any,
    };
  }
}
