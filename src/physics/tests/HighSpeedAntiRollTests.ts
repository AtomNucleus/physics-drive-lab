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
  // A smooth ISO-style left/right transient rather than an impossible steering step.
  // Peak handwheel demand is deliberately modest because 200 km/h multiplies even
  // a small road-wheel angle into a large lateral acceleration.
  const pulseDuration = 0.72;
  if (time < pulseDuration) {
    return 0.18 * Math.sin(Math.PI * time / pulseDuration);
  }
  if (time < pulseDuration * 2) {
    const local = time - pulseDuration;
    return -0.18 * Math.sin(Math.PI * local / pulseDuration);
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
  scenario: '200 km/h smooth double lane change on flat dry surface',
  currentBars,
  noBars,
  rollReductionFraction: noBars.peakRollDeg > 1e-6
    ? 1 - currentBars.peakRollDeg / noBars.peakRollDeg
    : 0,
}, null, 2));

// Initial diagnostic gates are intentionally broad. The point of this test is to
// expose high-speed anti-roll behavior numerically before tightening realism limits.
assert(currentBars.peakRollDeg < 12, `anti-roll model allowed extreme body roll: ${currentBars.peakRollDeg.toFixed(2)} deg`);
assert(currentBars.peakBodyRiseM < 0.20, `anti-roll model jacked the body upward by ${currentBars.peakBodyRiseM.toFixed(3)} m`);
assert(currentBars.peakBodyDropM < 0.20, `anti-roll model dropped the body by ${currentBars.peakBodyDropM.toFixed(3)} m`);
assert(currentBars.finalSpeedKmh > 100, `lane-change test lost implausible speed: ${currentBars.finalSpeedKmh.toFixed(1)} km/h`);
