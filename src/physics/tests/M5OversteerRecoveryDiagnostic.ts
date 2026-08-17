import assert from 'node:assert/strict';
import { Simulation } from '../Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { PhysicsMath } from '../math/PhysicsMath';
import { updateDigitalSteeringInput } from '../DigitalSteeringInput';

const dt = 1 / 120;
const DEG = 180 / Math.PI;
const neutral = { throttle: 0, brake: 0, steer: 0, handbrake: false, shiftUp: false, shiftDown: false };
const sideslipDeg = (sim: Simulation) => {
  const v = sim.vehicle.rigidBody.getLocalVelocity();
  return Math.atan2(v.x, Math.max(0.1, Math.abs(v.z))) * DEG;
};
function sample(sim: Simulation, t: number, driverInput: number) {
  const s = sim.vehicle.getState();
  const steering = sim.suspensionKinematics.steeringDynamics.telemetry;
  const avg = (a: number, b: number) => (a + b) * 0.5;
  return { t, speedKmh: s.speedKmh, yawRateDegS: s.yawRate * DEG, sideslipDeg: sideslipDeg(sim), driverInput, steerDeg: s.actualSteerAngle * DEG, steeringWheelDeg: steering.steeringWheelAngleRad * DEG, rackRateDegS: steering.rackAngularVelocityRadS * DEG, tireSatNm: steering.torques.tireSelfAligningRoadNm, mechanicalTrailNm: steering.torques.casterMechanicalTrailRoadNm, epsAssistNm: steering.torques.epsAssistSteeringWheelNm, rackTorqueNm: steering.torques.netRackRoadNm, frontSlipDeg: avg(s.wheels[0].slipAngle, s.wheels[1].slipAngle) * DEG, rearSlipDeg: avg(s.wheels[2].slipAngle, s.wheels[3].slipAngle) * DEG, frontKappa: avg(s.wheels[0].slipRatio, s.wheels[1].slipRatio), rearKappa: avg(s.wheels[2].slipRatio, s.wheels[3].slipRatio), frontFyN: s.wheels[0].forceVectorLat + s.wheels[1].forceVectorLat, rearFyN: s.wheels[2].forceVectorLat + s.wheels[3].forceVectorLat };
}
function makeOversteeringM5() {
  const config = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES, absMode: 'OFF', tcsMode: 'OFF' } as any;
  assert.equal(config.absMode, 'OFF');
  assert.equal(config.tcsMode, 'OFF');
  assert.equal(config.driftAssist ?? 0, 0, 'recovery test must not enable drift assist');
  const sim = new Simulation(config);
  sim.reset(0, 0, 0);
  const lf = config.wheelbase * (1 - config.weightDistributionFront);
  const lr = config.wheelbase * config.weightDistributionFront;
  const expectedYawInertia = config.mass * lf * lr;
  const initialYawInertia = sim.vehicle.rigidBody.config.inertia.y;
  assert(Math.abs(initialYawInertia - expectedYawInertia) / expectedYawInertia < 1e-9, `M5 yaw inertia does not match axle/CG model at construction: actual=${initialYawInertia.toFixed(1)}, expected=${expectedYawInertia.toFixed(1)} kg*m^2`);
  assert(initialYawInertia > 4800 && initialYawInertia < 6000, `M5 yaw inertia outside plausible heavy-sedan guardrail: ${initialYawInertia.toFixed(1)} kg*m^2`);
  sim.setConfig(config);
  const reconfiguredYawInertia = sim.vehicle.rigidBody.config.inertia.y;
  assert(Math.abs(reconfiguredYawInertia - expectedYawInertia) / expectedYawInertia < 1e-9, `M5 runtime reconfiguration changed yaw inertia: actual=${reconfiguredYawInertia.toFixed(1)}, expected=${expectedYawInertia.toFixed(1)} kg*m^2`);
  assert.equal(reconfiguredYawInertia, initialYawInertia, 'M5 yaw inertia must survive identical runtime config reapply');
  for (let i = 0; i < 300; i++) sim.stepExplicit(neutral, 1);
  const speedMs = 25;
  sim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, speedMs);
  sim.vehicle.wheels.forEach((wheel) => wheel.reset(speedMs));
  for (let i = 0; i < 60; i++) sim.stepExplicit(neutral, 1);
  for (let i = 0; i < 90; i++) sim.stepExplicit({ ...neutral, steer: 0.18 }, 1);
  let inductionSteps = 0;
  for (; inductionSteps < 120; inductionSteps++) {
    sim.stepExplicit({ ...neutral, steer: 0.18, handbrake: true }, 1);
    const state = sim.vehicle.getState();
    const rearSlip = 0.5 * (Math.abs(state.wheels[2].slipAngle) + Math.abs(state.wheels[3].slipAngle));
    if (Math.abs(state.yawRate) > 0.55 && rearSlip > 0.20) break;
  }
  return { sim, inductionSec: inductionSteps * dt, yawInertiaKgM2: reconfiguredYawInertia };
}
function runDigitalDriverRecovery() {
  const { sim, inductionSec, yawInertiaKgM2 } = makeOversteeringM5();
  let digitalInput = 0.18;
  let released = false;
  let releaseTimeSec: number | null = null;
  let peakCounterInput = 0;
  let peakCounterSteerDeg = 0;
  let peakRackRateDegS = 0;
  let peakRackTorqueNm = 0;
  const samples = [sample(sim, 0, digitalInput)];
  const wanted = new Set([0.10, 0.25, 0.50, 0.75, 1.00, 1.50].map((t) => Math.round(t / dt)));
  const totalSteps = Math.round(1.5 / dt);
  for (let i = 1; i <= totalSteps; i++) {
    const stateBefore = sim.vehicle.getState();
    const yawDegS = stateBefore.yawRate * DEG;
    if (!released && yawDegS <= 8) { released = true; releaseTimeSec = i * dt; }
    const direction: -1 | 0 = released ? 0 : -1;
    digitalInput = updateDigitalSteeringInput(digitalInput, direction, stateBefore.speedMs, dt);
    peakCounterInput = Math.max(peakCounterInput, -digitalInput);
    const state = sim.stepExplicit({ ...neutral, steer: digitalInput }, 1);
    const steering = sim.suspensionKinematics.steeringDynamics.telemetry;
    peakCounterSteerDeg = Math.max(peakCounterSteerDeg, -state.actualSteerAngle * DEG);
    peakRackRateDegS = Math.max(peakRackRateDegS, Math.abs(steering.rackAngularVelocityRadS * DEG));
    peakRackTorqueNm = Math.max(peakRackTorqueNm, Math.abs(steering.torques.netRackRoadNm));
    if (wanted.has(i)) samples.push(sample(sim, i * dt, digitalInput));
  }
  return { inductionSec, yawInertiaKgM2, releaseTimeSec, peakCounterInput, peakCounterSteerDeg, peakRackRateDegS, peakRackTorqueNm, samples };
}
const result = runDigitalDriverRecovery();
const at = (seconds: number) => {
  const found = result.samples.find((s) => Math.abs(s.t - seconds) < dt * 0.51);
  assert(found, `missing ${seconds}s recovery sample`);
  return found;
};
const start = at(0), t100 = at(0.10), t250 = at(0.25), t500 = at(0.50), t750 = at(0.75), t1000 = at(1.00);
console.log(JSON.stringify({ scenario: 'M5 oversteer catch using real keyboard/touch steering path; ABS/TCS OFF', ...result }, null, 2));
if (process.env.GITHUB_ACTIONS) {
  const summary = {
    inductionSec: result.inductionSec,
    releaseTimeSec: result.releaseTimeSec,
    peakCounterInput: result.peakCounterInput,
    peakCounterSteerDeg: result.peakCounterSteerDeg,
    peakRackRateDegS: result.peakRackRateDegS,
    peakRackTorqueNm: result.peakRackTorqueNm,
    start: { yaw: start.yawRateDegS, slip: start.sideslipDeg, rearSlip: start.rearSlipDeg, rearKappa: start.rearKappa },
    t100: { yaw: t100.yawRateDegS, slip: t100.sideslipDeg, rearKappa: t100.rearKappa, rearFyN: t100.rearFyN, steer: t100.steerDeg },
    t250: { yaw: t250.yawRateDegS, slip: t250.sideslipDeg, steer: t250.steerDeg },
    t500: { yaw: t500.yawRateDegS, slip: t500.sideslipDeg, steer: t500.steerDeg },
    t750: { yaw: t750.yawRateDegS, slip: t750.sideslipDeg, steer: t750.steerDeg },
    t1000: { yaw: t1000.yawRateDegS, slip: t1000.sideslipDeg, steer: t1000.steerDeg },
  };
  console.log(`::notice title=M5 oversteer recovery telemetry::${JSON.stringify(summary)}`);
}
assert(start.yawRateDegS > 45, `induced yaw is too mild: ${start.yawRateDegS.toFixed(1)} deg/s`);
assert(Math.abs(start.rearSlipDeg) > 10, `rear tire is not genuinely saturated: ${start.rearSlipDeg.toFixed(1)} deg`);
assert(Math.abs(start.rearKappa) > 0.5, `rear wheel lock trigger is too mild: kappa=${start.rearKappa.toFixed(3)}`);
assert(Math.abs(t100.rearKappa) < 0.05, `rear longitudinal slip did not recover: ${t100.rearKappa.toFixed(3)}`);
assert(Math.abs(t100.rearFyN) > 7000, `rear lateral force did not recover: ${t100.rearFyN.toFixed(0)} N`);
assert(result.peakCounterInput > 0.8, `digital input still withholds opposite lock: ${result.peakCounterInput.toFixed(3)}`);
assert(result.peakCounterSteerDeg > 20, `physical countersteer authority is too small: ${result.peakCounterSteerDeg.toFixed(1)} deg`);
assert(result.releaseTimeSec !== null && result.releaseTimeSec < 0.40, `driver could not arrest yaw promptly; release=${result.releaseTimeSec}`);
assert(Math.abs(t250.yawRateDegS) < Math.abs(start.yawRateDegS) * 0.30, `countersteer did not arrest yaw by 250 ms: ${t250.yawRateDegS.toFixed(1)} deg/s`);
assert(Math.abs(t500.yawRateDegS) < Math.abs(start.yawRateDegS) * 0.60, `opposite-yaw unwind became a snap spin: ${t500.yawRateDegS.toFixed(1)} deg/s`);
assert(Math.abs(t750.yawRateDegS) < 5, `yaw not under control by 750 ms: ${t750.yawRateDegS.toFixed(1)} deg/s`);
assert(Math.abs(t750.sideslipDeg) < 3, `body sideslip not caught by 750 ms: ${t750.sideslipDeg.toFixed(1)} deg`);
assert(Math.abs(t1000.yawRateDegS) < 2, `yaw did not settle by 1 s: ${t1000.yawRateDegS.toFixed(1)} deg/s`);
assert(Math.abs(t1000.sideslipDeg) < 2, `sideslip did not settle by 1 s: ${t1000.sideslipDeg.toFixed(1)} deg`);
console.log('M5OversteerRecoveryTests: PASS');
