import assert from 'node:assert/strict';
import type { ControlInputs, VehicleConfig } from '../../types';
import { Simulation } from '../Simulation';
import { ProvingGroundSurfaceProvider } from '../SurfaceProvider';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { updateDigitalSteeringInput } from '../DigitalSteeringInput';

const DT = 1 / 120;
const START_SPEED_MS = 50 / 3.6;
const DEG = 180 / Math.PI;

const zeroInputs: ControlInputs = {
  throttle: 0,
  brake: 0,
  steer: 0,
  handbrake: false,
  shiftUp: false,
  shiftDown: false,
};

const config = {
  ...DEFAULT_VEHICLE_CONFIG,
  ...BMW_M5_2025_OVERRIDES,
} as VehicleConfig;

function makeRollingM5() {
  const sim = new Simulation(config, new ProvingGroundSurfaceProvider());
  sim.reset(0, 0, 0);
  sim.vehicle.powertrain.isAutomatic = false;
  sim.vehicle.powertrain.gear = 0;
  sim.vehicle.rigidBody.velocity.z = START_SPEED_MS;
  for (const wheel of sim.vehicle.wheels) wheel.angularVelocity = START_SPEED_MS / config.wheelRadius;
  for (let i = 0; i < 60; i++) sim.stepExplicit(zeroInputs, 1);
  return sim;
}

type SteerProvider = (sim: Simulation) => number;

function runTurn(label: string, steerProvider: SteerProvider, durationSec = 2.0) {
  const sim = makeRollingM5();
  const radii: number[] = [];
  const geometricRadii: number[] = [];
  const speeds: number[] = [];
  let peakLatG = 0;
  let peakFrontSlipDeg = 0;
  let peakSteerInput = 0;
  let lateMeanFrontSteerDeg = 0;
  let lateSamples = 0;

  const steps = Math.round(durationSec / DT);
  for (let step = 0; step < steps; step++) {
    const steer = steerProvider(sim);
    peakSteerInput = Math.max(peakSteerInput, Math.abs(steer));
    const state = sim.stepExplicit({ ...zeroInputs, steer }, 1);
    const localVelocity = sim.vehicle.rigidBody.getLocalVelocity();
    const speed = Math.hypot(localVelocity.x, localVelocity.z);
    const yawRate = Math.abs(sim.vehicle.rigidBody.getLocalAngularVelocity().y);
    const frontSteer = Math.abs((state.wheels[0].steerAngle + state.wheels[1].steerAngle) * 0.5);

    peakLatG = Math.max(peakLatG, Math.abs(state.lateralG));
    peakFrontSlipDeg = Math.max(
      peakFrontSlipDeg,
      Math.abs(state.wheels[0].slipAngle) * DEG,
      Math.abs(state.wheels[1].slipAngle) * DEG
    );

    if (step > 90) {
      speeds.push(speed);
      if (yawRate > 0.03) radii.push(speed / yawRate);
      if (frontSteer > 0.005) geometricRadii.push(config.wheelbase / Math.tan(frontSteer));
      lateMeanFrontSteerDeg += frontSteer * DEG;
      lateSamples++;
    }
  }

  const mean = (values: number[]) => values.reduce((sum, value) => sum + value, 0) / Math.max(1, values.length);
  return {
    label,
    peakSteerInput,
    lateMeanSpeedKmh: mean(speeds) * 3.6,
    lateMeanFrontSteerDeg: lateSamples ? lateMeanFrontSteerDeg / lateSamples : 0,
    lateMeanYawRadiusM: mean(radii),
    lateMeanGeometricRadiusM: mean(geometricRadii),
    peakLatG,
    peakFrontSlipDeg,
  };
}

let shapedInput = 0;
const shapedDigital = runTurn('held keyboard/touch full-left', (sim) => {
  const speed = Math.abs(sim.vehicle.rigidBody.getLocalVelocity().z);
  shapedInput = updateDigitalSteeringInput(shapedInput, 1, speed, DT);
  return shapedInput;
});

const rawRack = runTurn('raw steer input +1', () => 1);

console.log(JSON.stringify({
  scenario: 'BMW M5 G90 50 km/h full-left turn-radius diagnostic',
  shapedDigital,
  rawRack,
  theoreticalGripRadiusAt1gM: START_SPEED_MS * START_SPEED_MS / 9.81,
}, null, 2));

assert(Number.isFinite(shapedDigital.lateMeanYawRadiusM), 'digital turn radius must be finite');
assert(Number.isFinite(rawRack.lateMeanYawRadiusM), 'raw-rack turn radius must be finite');
console.log('TurnRadiusTests: PASS');
