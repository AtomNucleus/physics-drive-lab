/**
 * Vehicle Physics 2.0 Core Engine
 * 14-DOF Deterministic Fixed-Step Simulation Architecture
 */

import { VehicleConfig, VehicleState, ControlInputs } from '../types';
import { Simulation } from './Simulation';
import { Vehicle } from './Vehicle';
import { ProvingGroundSurfaceProvider, SurfaceSample } from './SurfaceProvider';
import { HeadlessTestRunner, TestResult } from './tests/HeadlessTestRunner';

export class VehiclePhysicsEngine {
  public simulation: Simulation;
  public surfaceProvider: ProvingGroundSurfaceProvider;

  constructor(config: VehicleConfig) {
    this.surfaceProvider = new ProvingGroundSurfaceProvider();
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
    const snapshot = this.simulation.vehicle.getState();

    // App/UI code historically toggles automatic mode through `engine.state`.
    // VehicleState is otherwise a telemetry snapshot, so define this one field as
    // a live accessor to the actual powertrain instead of silently mutating a copy.
    Object.defineProperty(snapshot, 'isAutomatic', {
      enumerable: true,
      configurable: true,
      get: () => this.simulation.vehicle.powertrain.isAutomatic,
      set: (enabled: boolean) => {
        this.simulation.vehicle.powertrain.isAutomatic = Boolean(enabled);
      },
    });

    return snapshot;
  }

  public setAutomaticTransmission(enabled: boolean) {
    this.simulation.vehicle.powertrain.isAutomatic = Boolean(enabled);
  }

  public toggleAutomaticTransmission(): boolean {
    const next = !this.simulation.vehicle.powertrain.isAutomatic;
    this.simulation.vehicle.powertrain.isAutomatic = next;
    return next;
  }

  public setConfig(newConfig: VehicleConfig) {
    this.simulation.setConfig(newConfig);
  }

  public reset(x: number = 0, z: number = 0, yaw: number = 0) {
    this.simulation.reset(x, z, yaw);
  }

  public triggerClutchKick() {
    this.simulation.vehicle.triggerClutchKick();
  }

  public toggleDrs() {
    this.simulation.vehicle.toggleDrs();
  }

  public sampleTerrainAndSurface(x: number, z: number): SurfaceSample {
    return this.surfaceProvider.sampleSurface(x, z);
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
