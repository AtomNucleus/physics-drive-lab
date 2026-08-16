/**
 * Vehicle Physics 2.0 Core Engine
 * 14-DOF Deterministic Fixed-Step Simulation Architecture
 */

import { VehicleConfig, VehicleState, ControlInputs } from '../types';
import { Simulation } from './Simulation';
import { Vehicle } from './Vehicle';
import { ISurfaceProvider, ProvingGroundSurfaceProvider, SurfaceSample } from './SurfaceProvider';
import { HeadlessTestRunner, TestResult } from './tests/HeadlessTestRunner';

export class VehiclePhysicsEngine {
  public simulation: Simulation;
  public surfaceProvider: ISurfaceProvider;

  constructor(config: VehicleConfig, surfaceProvider: ISurfaceProvider = new ProvingGroundSurfaceProvider()) {
    this.surfaceProvider = surfaceProvider;
    this.simulation = new Simulation(config, this.surfaceProvider);
  }

  public get config(): VehicleConfig {
    return this.simulation.vehicle.config;
  }

  public set config(cfg: VehicleConfig) {
    // Route property assignment through Simulation as well so mass, CG and
    // principal inertias are updated together with the subsystem tuning.
    this.simulation.setConfig(cfg);
  }

  public get state(): VehicleState {
    return this.simulation.vehicle.getState();
  }

  public setConfig(newConfig: VehicleConfig) {
    this.simulation.setConfig(newConfig);
  }

  /**
   * Swap the world surface without reconstructing the vehicle. The proving
   * ground remains the default; imported tracks can install a mesh-backed
   * provider after their package has loaded.
   */
  public setSurfaceProvider(surfaceProvider: ISurfaceProvider) {
    this.surfaceProvider = surfaceProvider;
    this.simulation.vehicle.surfaceProvider = surfaceProvider;
  }

  public reset(x: number = 0, z: number = 0, yaw: number = 0) {
    this.simulation.reset(x, z, yaw);

    // Vehicle.reset uses a ground-relative ride-height convention. Move that
    // baseline onto the actual imported road so a mountain spawn does not begin
    // hundreds of metres below or above the local surface.
    const surface = this.surfaceProvider.sampleSurface(x, z, this.simulation.vehicle.rigidBody.position.y);
    if (Number.isFinite(surface.elevation) && surface.elevation > -1000) {
      this.simulation.vehicle.rigidBody.position.y += surface.elevation;

      // Refresh Simulation's interpolation snapshots after changing Y without
      // advancing physics time.
      this.simulation.stepExplicit(
        {
          throttle: 0,
          brake: 0,
          steer: 0,
          handbrake: false,
          shiftUp: false,
          shiftDown: false,
        },
        0
      );
    }
  }

  public triggerClutchKick() {
    this.simulation.vehicle.triggerClutchKick();
  }

  public toggleDrs() {
    this.simulation.vehicle.toggleDrs();
  }

  public sampleTerrainAndSurface(x: number, z: number, referenceY?: number): SurfaceSample {
    return this.surfaceProvider.sampleSurface(x, z, referenceY);
  }

  /** Advance the 120 Hz fixed accumulator physics with state interpolation. */
  public update(deltaTime: number, inputs: ControlInputs): VehicleState {
    return this.simulation.advance(deltaTime, inputs);
  }

  public runHeadlessTests(): TestResult[] {
    return HeadlessTestRunner.runAllTests(this.config);
  }
}

export { Simulation } from './Simulation';
export { Vehicle } from './Vehicle';
export { RigidBody } from './RigidBody';
export { SuspensionSystem } from './Suspension';
export { TireModel } from './TireModel';
export { WheelDynamics } from './WheelDynamics';
export { Powertrain } from './Powertrain';
export { DifferentialSystem } from './Differential';
export { BrakeSystem } from './Brakes';
export { DriverAidsSystem } from './DriverAids';
export { AerodynamicsSystem } from './Aero';
export { TelemetrySystem } from './Telemetry';
export { ProvingGroundSurfaceProvider } from './SurfaceProvider';
export { HeadlessTestRunner } from './tests/HeadlessTestRunner';
export * from './math/PhysicsMath';
