import assert from 'node:assert/strict';
import type { ControlInputs, VehicleConfig } from '../../types';
import { Simulation } from '../Simulation';
import { ProvingGroundSurfaceProvider } from '../SurfaceProvider';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { updateDigitalSteeringInput } from '../DigitalSteeringInput';

const DT = 1 / 120;
const START_SPEED_MS = 30 / 3.6;
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

type SteerProvider = (sim: Simulation, step: number) => number;

function runCorner(label: string, steerProvider: SteerProvider, durationSec: number) {
  const sim = makeRollingM5();
  let peakFrontSlipDeg = 0;
  let peakRearSlipDeg = 0;
  let peakLatG = 0;
  let skidSamples = 0;
  let frontSkidSamples = 0;
  let rearSkidSamples = 0;
  let yawRatioSum = 0;
  let yawRatioSamples = 0;
  let meanFrontSteerDeg = 0;
  let finalSpeedKmh = 0;
  let peakSteerInput = 0;

  const totalSteps = Math.round(durationSec / DT);
  for (let step = 0; step < totalSteps; step++) {
    const steerInput = steerProvider(sim, step);
    peakSteerInput = Math.max(peakSteerInput, Math.abs(steerInput));
    const state = sim.stepExplicit({ ...zeroInputs, steer: steerInput }, 1);
    const frontSteer = (state.wheels[0].steerAngle + state.wheels[1].steerAngle) * 0.5;
    meanFrontSteerDeg = Math.abs(frontSteer) * DEG;
    finalSpeedKmh = state.speedKmh;

    const frontSlip = Math.max(Math.abs(state.wheels[0].slipAngle), Math.abs(state.wheels[1].slipAngle)) * DEG;
    const rearSlip = Math.max(Math.abs(state.wheels[2].slipAngle), Math.abs(state.wheels[3].slipAngle)) * DEG;
    peakFrontSlipDeg = Math.max(peakFrontSlipDeg, frontSlip);
    peakRearSlipDeg = Math.max(peakRearSlipDeg, rearSlip);
    peakLatG = Math.max(peakLatG, Math.abs(state.lateralG));

    const frontSkid = state.wheels[0].isSkidding || state.wheels[1].isSkidding;
    const rearSkid = state.wheels[2].isSkidding || state.wheels[3].isSkidding;
    if (frontSkid || rearSkid) skidSamples++;
    if (frontSkid) frontSkidSamples++;
    if (rearSkid) rearSkidSamples++;

    if (step > 60 && Math.abs(frontSteer) > 0.01) {
      const localSpeed = Math.abs(sim.vehicle.rigidBody.getLocalVelocity().z);
      const targetYawRate = localSpeed * Math.tan(frontSteer) / config.wheelbase;
      const actualYawRate = sim.vehicle.rigidBody.getLocalAngularVelocity().y;
      if (Math.abs(targetYawRate) > 0.05) {
        yawRatioSum += Math.abs(actualYawRate / targetYawRate);
        yawRatioSamples++;
      }
    }
  }

  return {
    label,
    peakSteerInput,
    meanFrontSteerDeg,
    peakFrontSlipDeg,
    peakRearSlipDeg,
    peakLatG,
    skidSamples,
    frontSkidSamples,
    rearSkidSamples,
    meanYawResponseRatio: yawRatioSamples ? yawRatioSum / yawRatioSamples : 0,
    finalSpeedKmh,
  };
}

// With the physical rack no longer shrinking at speed, 40% input produces roughly
// the same ~13-degree road-wheel angle that the old 45% speed-capped case exercised.
// Keep this as the ordinary tight-road-turn handling check; it should stay below
// gross front sliding and build the expected yaw without relying on a hidden cap.
const moderate = runCorner('moderate-raw', () => 0.40, 2.0);

// Full steering is intentionally also visible. A simulator must allow the driver
// to over-command the front tires; it must not silently cap steering to protect
// them from understeer.
const rawFullDigital = runCorner('raw-full-digital', () => 1.0, 1.2);

let digitalInput = 0;
const heldDigital = runCorner(
  'held-digital-left',
  (sim) => {
    const speedMs = Math.abs(sim.vehicle.rigidBody.getLocalVelocity().z);
    digitalInput = updateDigitalSteeringInput(digitalInput, 1, speedMs, DT);
    return digitalInput;
  },
  2.0
);

console.log(JSON.stringify({ scenario: 'M5 low-speed cornering at ~30 km/h', moderate, rawFullDigital, heldDigital }, null, 2));

assert(moderate.peakFrontSlipDeg < 9.0, `moderate corner gross-slid front tires: ${moderate.peakFrontSlipDeg.toFixed(2)} deg`);
assert(moderate.frontSkidSamples === 0, `moderate corner emitted front skid state for ${moderate.frontSkidSamples} samples`);
assert(moderate.meanYawResponseRatio > 0.62, `moderate corner yaw response too low: ${moderate.meanYawResponseRatio.toFixed(3)}`);
assert(moderate.peakRearSlipDeg < 9.0, `moderate corner gross-slid rear tires: ${moderate.peakRearSlipDeg.toFixed(2)} deg`);

assert(heldDigital.peakSteerInput > 0.99, `held digital steering must preserve full authority, got ${heldDigital.peakSteerInput.toFixed(3)}`);
assert(heldDigital.peakFrontSlipDeg < 30, `held full steering became numerically unstable: ${heldDigital.peakFrontSlipDeg.toFixed(2)} deg`);
assert(heldDigital.meanYawResponseRatio > 0.55, `held full steering lost plausible yaw response: ${heldDigital.meanYawResponseRatio.toFixed(3)}`);
assert(heldDigital.finalSpeedKmh > 15, `held full steering scrubbed implausibly much speed: ${heldDigital.finalSpeedKmh.toFixed(1)} km/h`);

console.log('LowSpeedCorneringTests: PASS');
