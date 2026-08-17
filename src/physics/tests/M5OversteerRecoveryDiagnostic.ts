import { Simulation } from '../Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { PhysicsMath } from '../math/PhysicsMath';

const dt = 1 / 120;
const neutral = { throttle: 0, brake: 0, steer: 0, handbrake: false, shiftUp: false, shiftDown: false };

const sideslipDeg = (sim: Simulation) => {
  const v = sim.vehicle.rigidBody.getLocalVelocity();
  return Math.atan2(v.x, Math.max(0.1, Math.abs(v.z))) * 180 / Math.PI;
};

function sample(sim: Simulation, t: number) {
  const s = sim.vehicle.getState();
  const avg = (a: number, b: number) => (a + b) * 0.5;
  return {
    t,
    speedKmh: s.speedKmh,
    yawRateDegS: s.yawRate * 180 / Math.PI,
    sideslipDeg: sideslipDeg(sim),
    steerDeg: s.actualSteerAngle * 180 / Math.PI,
    frontSlipDeg: avg(s.wheels[0].slipAngle, s.wheels[1].slipAngle) * 180 / Math.PI,
    rearSlipDeg: avg(s.wheels[2].slipAngle, s.wheels[3].slipAngle) * 180 / Math.PI,
    frontKappa: avg(s.wheels[0].slipRatio, s.wheels[1].slipRatio),
    rearKappa: avg(s.wheels[2].slipRatio, s.wheels[3].slipRatio),
    frontFyN: s.wheels[0].forceVectorLat + s.wheels[1].forceVectorLat,
    rearFyN: s.wheels[2].forceVectorLat + s.wheels[3].forceVectorLat,
    frontFxN: s.wheels[0].forceVectorLong + s.wheels[1].forceVectorLong,
    rearFxN: s.wheels[2].forceVectorLong + s.wheels[3].forceVectorLong,
  };
}

function run(label: string, yawInertia: number, counterDurationSec: number, counterSteer = -0.42) {
  const config = {
    ...DEFAULT_VEHICLE_CONFIG,
    ...BMW_M5_2025_OVERRIDES,
    absMode: 'OFF',
    tcsMode: 'OFF',
  } as any;
  const sim = new Simulation(config);
  sim.reset(0, 0, 0);
  for (let i = 0; i < 300; i++) sim.stepExplicit(neutral, 1);
  const speedMs = 25;
  sim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, speedMs);
  sim.vehicle.wheels.forEach((w) => w.reset(speedMs));
  sim.vehicle.rigidBody.config.inertia.y = yawInertia;
  for (let i = 0; i < 60; i++) sim.stepExplicit(neutral, 1);

  for (let i = 0; i < 90; i++) sim.stepExplicit({ ...neutral, steer: 0.18 }, 1);

  let inductionSteps = 0;
  for (; inductionSteps < 120; inductionSteps++) {
    sim.stepExplicit({ ...neutral, steer: 0.18, handbrake: true }, 1);
    const state = sim.vehicle.getState();
    const rearSlip = 0.5 * (Math.abs(state.wheels[2].slipAngle) + Math.abs(state.wheels[3].slipAngle));
    if (Math.abs(state.yawRate) > 0.55 && rearSlip > 0.20) break;
  }

  const samples = [sample(sim, 0)];
  const totalRecoverySec = 1.5;
  const totalSteps = Math.round(totalRecoverySec / dt);
  const wanted = new Set([0.10, 0.25, 0.50, 0.75, 1.00, 1.50].map((t) => Math.round(t / dt)));
  for (let i = 1; i <= totalSteps; i++) {
    const t = i * dt;
    sim.stepExplicit({ ...neutral, steer: t <= counterDurationSec ? counterSteer : 0 }, 1);
    if (wanted.has(i)) samples.push(sample(sim, t));
  }
  for (let i = 0; i < 120; i++) sim.stepExplicit(neutral, 1);

  return {
    label,
    yawInertia,
    counterDurationSec,
    counterSteer,
    inductionSec: inductionSteps * dt,
    samples,
    finalAfterExtra1s: sample(sim, totalRecoverySec + 1.0),
  };
}

const currentYawInertia = 2582.1142091360184;
console.log(JSON.stringify([
  run('current-short-counter', currentYawInertia, 0.30),
  run('current-medium-counter', currentYawInertia, 0.55),
  run('current-held-counter', currentYawInertia, 1.50),
  run('higher-inertia-medium-counter', 4100, 0.55),
], null, 2));
