import assert from 'node:assert/strict';
import type { ControlInputs, VehicleConfig } from '../../types';
import { Simulation } from '../Simulation';
import { ProvingGroundSurfaceProvider } from '../SurfaceProvider';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';

const DT = 1 / 120;
const START_SPEED_MS = 200 / 3.6;

const zeroInputs: ControlInputs = {
  throttle: 0,
  brake: 0,
  steer: 0,
  handbrake: false,
  shiftUp: false,
  shiftDown: false,
};

function makeM5(overrides: Partial<VehicleConfig> = {}) {
  const config = {
    ...DEFAULT_VEHICLE_CONFIG,
    ...BMW_M5_2025_OVERRIDES,
    ...overrides,
  } as VehicleConfig;

  const sim = new Simulation(config, new ProvingGroundSurfaceProvider());
  sim.reset(0, 0, 0);
  sim.vehicle.powertrain.isAutomatic = false;
  sim.vehicle.powertrain.gear = 0;
  sim.vehicle.rigidBody.velocity.z = START_SPEED_MS;
  for (const wheel of sim.vehicle.wheels) {
    wheel.angularVelocity = START_SPEED_MS / config.wheelRadius;
  }

  // Give the tire/suspension states time to initialize at speed before steering.
  for (let i = 0; i < 60; i++) sim.stepExplicit(zeroInputs, 1);
  return { sim, config };
}

function laneChangeSteer(time: number) {
  // At 200 km/h, the M5 steering limiter leaves roughly 13.3 deg of available
  // road-wheel lock. A 4% command therefore peaks near 0.53 deg, which is in the
  // neighborhood of a 1 g kinematic demand for a 3.0 m wheelbase instead of the
  // impossible multi-g demand created by the old 18% diagnostic pulse.
  const pulseDuration = 0.72;
  if (time < pulseDuration) {
    return 0.04 * Math.sin(Math.PI * time / pulseDuration);
  }
  if (time < pulseDuration * 2) {
    const local = time - pulseDuration;
    return -0.04 * Math.sin(Math.PI * local / pulseDuration);
  }
  return 0;
}

function runHighSpeedLaneChange(overrides: Partial<VehicleConfig> = {}) {
  const { sim, config } = makeM5({ antiRollCrossCoupling: 0, ...overrides });
  const startY = sim.vehicle.rigidBody.position.y;

  let peakRollDeg = 0;
  let peakRollRateDegPerSec = 0;
  let peakLatG = 0;
  let peakVerticalG = 0;
  let peakActualSteerDeg = 0;
  let peakBodyRiseM = 0;
  let peakBodyDropM = 0;
  let peakTotalNormalLoadN = 0;
  let minimumTotalNormalLoadN = Number.POSITIVE_INFINITY;
  let minimumWheelLoadN = Number.POSITIVE_INFINITY;
  let peakArbForceN = 0;
  let peakNetSuspensionBiasN = 0;
  let airborneSamples = 0;

  for (let step = 0; step < 120 * 3.0; step++) {
    const t = step * DT;
    const inputs: ControlInputs = { ...zeroInputs, steer: laneChangeSteer(t) };
    const state = sim.stepExplicit(inputs, 1);
    const euler = sim.vehicle.rigidBody.getEuler();
    const localAngularVelocity = sim.vehicle.rigidBody.getLocalAngularVelocity();
    const susp = sim.vehicle.suspension.states;
    const loads = susp.map((corner) => corner.tireNormalForceN);
    const totalNormalLoad = loads.reduce((sum, load) => sum + load, 0);

    peakRollDeg = Math.max(peakRollDeg, Math.abs(euler.roll) * 180 / Math.PI);
    peakRollRateDegPerSec = Math.max(
      peakRollRateDegPerSec,
      Math.abs(localAngularVelocity.z) * 180 / Math.PI
    );
    peakLatG = Math.max(peakLatG, Math.abs(state.lateralG));
    peakVerticalG = Math.max(peakVerticalG, Math.abs(state.verticalG));
    peakActualSteerDeg = Math.max(peakActualSteerDeg, Math.abs(state.actualSteerAngle) * 180 / Math.PI);
    peakBodyRiseM = Math.max(peakBodyRiseM, sim.vehicle.rigidBody.position.y - startY);
    peakBodyDropM = Math.max(peakBodyDropM, startY - sim.vehicle.rigidBody.position.y);
    peakTotalNormalLoadN = Math.max(peakTotalNormalLoadN, totalNormalLoad);
    minimumTotalNormalLoadN = Math.min(minimumTotalNormalLoadN, totalNormalLoad);
    minimumWheelLoadN = Math.min(minimumWheelLoadN, ...loads);
    peakArbForceN = Math.max(
      peakArbForceN,
      ...susp.map((corner) => Math.abs(corner.antiRollBarForceN))
    );

    // The mechanical ARBs are internal equal-and-opposite forces. On a flat road,
    // they must not create a net vertical chassis reaction. A positive residual here
    // exposes force clipping / jacking in the suspension solver.
    const baseInternalForce = susp.reduce(
      (sum, corner) => sum + corner.springForceN + corner.damperForceN +
        corner.bumpStopForceN + corner.hardStopForceN,
      0
    );
    const netSuspensionBias = susp.reduce((sum, corner) => sum + corner.chassisForceN, 0) - baseInternalForce;
    peakNetSuspensionBiasN = Math.max(peakNetSuspensionBiasN, Math.abs(netSuspensionBias));

    if (susp.some((corner) => corner.isAirborne)) airborneSamples++;

    assert(Number.isFinite(state.speedKmh), 'high-speed lane change produced non-finite speed');
    assert(Number.isFinite(euler.roll), 'high-speed lane change produced non-finite roll');
  }

  const final = sim.vehicle.getState();
  const weightN = config.mass * 9.81;

  return {
    peakRollDeg,
    peakRollRateDegPerSec,
    peakLatG,
    peakVerticalG,
    peakActualSteerDeg,
    peakBodyRiseM,
    peakBodyDropM,
    peakTotalNormalLoadN,
    minimumTotalNormalLoadN,
    minimumWheelLoadN,
    peakArbForceN,
    peakNetSuspensionBiasN,
    airborneSamples,
    weightN,
    finalSpeedKmh: final.speedKmh,
    finalRollDeg: Math.abs(final.roll) * 180 / Math.PI,
  };
}

const currentBars = runHighSpeedLaneChange();
const noBars = runHighSpeedLaneChange({
  rollStiffnessFront: 0,
  rollStiffnessRear: 0,
});

console.log(JSON.stringify({
  scenario: '200 km/h smooth ~1 g double lane change on flat dry surface',
  currentBars,
  noBars,
  rollReductionFraction: noBars.peakRollDeg > 1e-6
    ? 1 - currentBars.peakRollDeg / noBars.peakRollDeg
    : 0,
}, null, 2));

// Broad safety gates for the diagnostic pass. These become tighter once the
// force-conservation issue is corrected and the realistic response is measured.
assert(currentBars.peakLatG > 0.45, `high-speed maneuver was too mild to exercise roll: ${currentBars.peakLatG.toFixed(2)} g`);
assert(currentBars.peakRollDeg < 8, `anti-roll model allowed excessive body roll: ${currentBars.peakRollDeg.toFixed(2)} deg`);
assert(currentBars.peakBodyRiseM < 0.10, `anti-roll model jacked the body upward by ${currentBars.peakBodyRiseM.toFixed(3)} m`);
assert(currentBars.peakBodyDropM < 0.10, `anti-roll model dropped the body by ${currentBars.peakBodyDropM.toFixed(3)} m`);
assert(currentBars.finalSpeedKmh > 130, `lane-change test lost implausible speed: ${currentBars.finalSpeedKmh.toFixed(1)} km/h`);
