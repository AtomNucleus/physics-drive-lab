import assert from 'node:assert/strict';
import type { ControlInputs, VehicleConfig } from '../../types';
import { Simulation } from '../Simulation';
import { ProvingGroundSurfaceProvider } from '../SurfaceProvider';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';

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
  for (const wheel of sim.vehicle.wheels) {
    wheel.angularVelocity = START_SPEED_MS / config.wheelRadius;
  }

  // Give suspension/tire transient states time to settle while preserving the
  // rolling test speed closely enough for a 30 km/h neighborhood diagnostic.
  for (let i = 0; i < 60; i++) sim.stepExplicit(zeroInputs, 1);
  return sim;
}

function runCorner(steerInput: number, durationSec: number) {
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

  const totalSteps = Math.round(durationSec / DT);
  for (let step = 0; step < totalSteps; step++) {
    const state = sim.stepExplicit({ ...zeroInputs, steer: steerInput }, 1);
    const frontSteer = (state.wheels[0].steerAngle + state.wheels[1].steerAngle) * 0.5;
    meanFrontSteerDeg = Math.abs(frontSteer) * DEG;
    finalSpeedKmh = state.speedKmh;

    const frontSlip = Math.max(
      Math.abs(state.wheels[0].slipAngle),
      Math.abs(state.wheels[1].slipAngle)
    ) * DEG;
    const rearSlip = Math.max(
      Math.abs(state.wheels[2].slipAngle),
      Math.abs(state.wheels[3].slipAngle)
    ) * DEG;
    peakFrontSlipDeg = Math.max(peakFrontSlipDeg, frontSlip);
    peakRearSlipDeg = Math.max(peakRearSlipDeg, rearSlip);
    peakLatG = Math.max(peakLatG, Math.abs(state.lateralG));

    const frontSkid = state.wheels[0].isSkidding || state.wheels[1].isSkidding;
    const rearSkid = state.wheels[2].isSkidding || state.wheels[3].isSkidding;
    if (frontSkid || rearSkid) skidSamples++;
    if (frontSkid) frontSkidSamples++;
    if (rearSkid) rearSkidSamples++;

    // After rack/transient settling, compare measured yaw rate to the geometric
    // bicycle-model demand at the *actual* front wheel angle. At a moderate turn
    // the heavy M5 may understeer somewhat, but it should not plow at a small
    // fraction of the requested yaw rate while its front tires gross-slide.
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
    steerInput,
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

// 45% steering at ~30 km/h corresponds to a normal tight road turn (~13 deg at
// the road wheels), not parking-lot full lock. It should remain below the gross
// sliding/smoke regime and build most of the yaw requested by its actual wheel angle.
const moderate = runCorner(0.45, 2.0);

// Full digital steering is intentionally logged as a diagnostic. A binary A/D key
// currently requests nearly full rack angle and can demand >1 g at 30 km/h; this
// scenario tells us how much of the user's smoke report comes from input shaping.
const fullDigital = runCorner(1.0, 1.2);

console.log(JSON.stringify({
  scenario: 'M5 low-speed cornering at ~30 km/h',
  moderate,
  fullDigital,
}, null, 2));

assert(
  moderate.peakFrontSlipDeg < 9.0,
  `moderate 30 km/h corner gross-slid front tires: ${moderate.peakFrontSlipDeg.toFixed(2)} deg`
);
assert(
  moderate.frontSkidSamples === 0,
  `moderate 30 km/h corner emitted front skid/smoke state for ${moderate.frontSkidSamples} samples`
);
assert(
  moderate.meanYawResponseRatio > 0.62,
  `moderate 30 km/h corner is excessively understeery: yaw response ratio ${moderate.meanYawResponseRatio.toFixed(3)}`
);
assert(
  moderate.peakRearSlipDeg < 9.0,
  `moderate 30 km/h corner gross-slid rear tires: ${moderate.peakRearSlipDeg.toFixed(2)} deg`
);

console.log('LowSpeedCorneringTests: PASS');
