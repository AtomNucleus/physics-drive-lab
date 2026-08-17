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

function run(yawInertia: number, rearSteerMaxDeg = 1.5) {
  const config = {
    ...DEFAULT_VEHICLE_CONFIG,
    ...BMW_M5_2025_OVERRIDES,
    absMode: 'OFF',
    tcsMode: 'OFF',
    rearSteerMaxDeg,
  } as any;
  const sim = new Simulation(config);
  sim.reset(0, 0, 0);
  for (let i = 0; i < 300; i++) sim.stepExplicit(neutral, 1);
  const speedMs = 25;
  sim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, speedMs);
  sim.vehicle.wheels.forEach((w) => w.reset(speedMs));
  sim.vehicle.rigidBody.config.inertia.y = yawInertia;
  for (let i = 0; i < 60; i++) sim.stepExplicit(neutral, 1);

  // Build a real cornering state first, then provoke rear saturation mechanically.
  for (let i = 0; i < 90; i++) sim.stepExplicit({ ...neutral, steer: 0.18 }, 1);

  let inductionSteps = 0;
  for (; inductionSteps < 120; inductionSteps++) {
    sim.stepExplicit({ ...neutral, steer: 0.18, handbrake: true }, 1);
    const state = sim.vehicle.getState();
    const rearSlip = 0.5 * (Math.abs(state.wheels[2].slipAngle) + Math.abs(state.wheels[3].slipAngle));
    if (Math.abs(state.yawRate) > 0.55 && rearSlip > 0.20) break;
  }

  const start = sim.vehicle.getState();
  const recovery: Array<{ t: number; yawRate: number; sideslip: number; rearSlip: number }> = [];
  for (let i = 0; i < 180; i++) {
    // No throttle, no brake, no stability system: only a driver countersteer input.
    sim.stepExplicit({ ...neutral, steer: -0.42 }, 1);
    const state = sim.vehicle.getState();
    recovery.push({
      t: (i + 1) * dt,
      yawRate: state.yawRate,
      sideslip: sideslipDeg(sim),
      rearSlip: 0.5 * (Math.abs(state.wheels[2].slipAngle) + Math.abs(state.wheels[3].slipAngle)),
    });
  }
  for (let i = 0; i < 120; i++) sim.stepExplicit(neutral, 1);

  const at = (sec: number) => recovery[Math.min(recovery.length - 1, Math.max(0, Math.round(sec / dt) - 1))];
  const final = sim.vehicle.getState();
  return {
    yawInertia,
    rearSteerMaxDeg,
    inductionSec: inductionSteps * dt,
    startSpeedKmh: start.speedKmh,
    startYawRateDegS: start.yawRate * 180 / Math.PI,
    startSideslipDeg: sideslipDeg(sim),
    startRearSlipDeg: 0.5 * (Math.abs(start.wheels[2].slipAngle) + Math.abs(start.wheels[3].slipAngle)) * 180 / Math.PI,
    at250ms: { yawRateDegS: at(0.25).yawRate * 180 / Math.PI, sideslipDeg: at(0.25).sideslip, rearSlipDeg: at(0.25).rearSlip * 180 / Math.PI },
    at500ms: { yawRateDegS: at(0.50).yawRate * 180 / Math.PI, sideslipDeg: at(0.50).sideslip, rearSlipDeg: at(0.50).rearSlip * 180 / Math.PI },
    at1000ms: { yawRateDegS: at(1.00).yawRate * 180 / Math.PI, sideslipDeg: at(1.00).sideslip, rearSlipDeg: at(1.00).rearSlip * 180 / Math.PI },
    maxRecoveryYawRateDegS: Math.max(...recovery.map((s) => Math.abs(s.yawRate))) * 180 / Math.PI,
    finalYawRateDegS: final.yawRate * 180 / Math.PI,
    finalSideslipDeg: sideslipDeg(sim),
    finalSpeedKmh: final.speedKmh,
  };
}

const currentYawInertia = 2582.1142091360184;
console.log(JSON.stringify([
  run(currentYawInertia),
  run(3600),
  run(4100),
  run(4600),
  run(4100, 0),
], null, 2));
