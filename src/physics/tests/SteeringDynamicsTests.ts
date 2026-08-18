import assert from 'node:assert/strict';
import { Simulation } from '../Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { PhysicsMath } from '../math/PhysicsMath';

const DT = 1 / 120;
const DEG = 180 / Math.PI;
const CONFIG = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES } as any;
const NEUTRAL = {
  throttle: 0,
  brake: 0,
  steer: 0,
  handbrake: false,
  shiftUp: false,
  shiftDown: false,
};

function makeSim(speedKmh = 0) {
  const sim = new Simulation(CONFIG);
  sim.reset(0, 0, 0);
  for (let i = 0; i < Math.round(2.2 / DT); i++) sim.stepExplicit(NEUTRAL as any, 1);
  if (speedKmh > 0) {
    const speedMs = speedKmh / 3.6;
    sim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, speedMs);
    sim.vehicle.wheels.forEach((wheel) => wheel.reset(speedMs));
  }
  return sim;
}

function run(sim: Simulation, seconds: number, steer: (t: number) => number) {
  const rows: any[] = [];
  for (let i = 0; i < Math.round(seconds / DT); i++) {
    const controls = { ...NEUTRAL, steer: steer(i * DT) };
    sim.stepExplicit(controls as any, 1);
    const state = sim.vehicle.getState() as any;
    const rack = sim.suspensionKinematics.steeringDynamics.telemetry;
    rows.push({
      t: (i + 1) * DT,
      speedKmh: state.speedKmh,
      yawRateDegS: state.yawRate * DEG,
      lateralG: state.lateralG,
      steeringWheelDeg: rack.steeringWheelAngleRad * DEG,
      targetSteeringWheelDeg: rack.targetSteeringWheelAngleRad * DEG,
      rackCenterDeg: rack.rackCenterAngleRad * DEG,
      rackRateDegS: rack.rackAngularVelocityRadS * DEG,
      flDeg: state.wheels[0].steerAngle * DEG,
      frDeg: state.wheels[1].steerAngle * DEG,
      leftComplianceDeg: rack.leftComplianceRad * DEG,
      rightComplianceDeg: rack.rightComplianceRad * DEG,
    });
  }
  return rows;
}

// Published G90 steering ratio + canonical Ackermann/mirror contract.
{
  const steering = makeSim().suspensionKinematics.steeringDynamics;
  assert.equal(steering.steeringRatioAt(0), 14.2, 'G90 physical steering must use the published 14.2:1 overall ratio');
  const left = steering.ackermannForCenter(0.20);
  const right = steering.ackermannForCenter(-0.20);
  assert(left.left > left.right && left.right > 0, 'left turn Ackermann ordering failed');
  assert(Math.abs(right.right) > Math.abs(right.left) && right.left < 0, 'right turn Ackermann ordering failed');
  assert(Math.abs(left.left + right.right) < 1e-10, 'inner Ackermann mirror failed');
  assert(Math.abs(left.right + right.left) < 1e-10, 'outer Ackermann mirror failed');
}

// A driver step must accelerate a finite rack; road-wheel angle cannot teleport.
{
  const sim = makeSim(100);
  const rows = run(sim, 0.8, () => 0.08);
  const first = rows[0];
  const at100ms = rows[Math.round(0.10 / DT) - 1];
  const late = rows.at(-1)!;
  assert(Math.abs(first.rackCenterDeg) < Math.abs(late.rackCenterDeg) * 0.5,
    `rack teleported on first step: first=${first.rackCenterDeg.toFixed(3)} late=${late.rackCenterDeg.toFixed(3)}`);
  assert(Math.abs(at100ms.rackCenterDeg) > Math.abs(first.rackCenterDeg) * 1.25,
    'rack did not progressively build angle after the first fixed step');
  assert(Math.max(...rows.map((row) => Math.abs(row.rackRateDegS))) <= CONFIG.steerSpeed * DEG + 0.5,
    'rack exceeded configured finite steering-rate authority');
}

// Compliance must be bounded and the physical rack, not the input adapter, must own release.
{
  const sim = makeSim(50);
  const held = run(sim, 1.2, () => 0.10);
  const heldAngle = Math.abs(held.at(-1)!.rackCenterDeg);
  const released = run(sim, 1.2, () => 0);
  assert(Math.abs(released[0].rackCenterDeg) > 0.01, 'road wheels snapped to center on release instead of unwinding dynamically');
  assert(Math.abs(released.at(-1)!.rackCenterDeg) < heldAngle * 0.40 + 0.25,
    'rack failed to substantially unwind after driver release');
  assert(Math.max(...[...held, ...released].flatMap((row) => [Math.abs(row.leftComplianceDeg), Math.abs(row.rightComplianceDeg)])) < 0.40,
    'steering compliance exceeded the intended sub-0.4 degree range');
}

// Mirrored inputs must produce mirrored steering and chassis response; this is an
// invariant, not a BMW target.
{
  const leftSim = makeSim(80);
  const rightSim = makeSim(80);
  const left = run(leftSim, 1.1, () => 0.07).at(-1)!;
  const right = run(rightSim, 1.1, () => -0.07).at(-1)!;
  const rel = (a: number, b: number) => Math.abs(a - b) / Math.max(1e-6, Math.max(Math.abs(a), Math.abs(b)));

  assert(left.steeringWheelDeg > 0 && right.steeringWheelDeg < 0, 'steering-wheel sign convention failed');
  assert(left.yawRateDegS > 0 && right.yawRateDegS < 0, 'yaw sign convention failed');
  assert(rel(Math.abs(left.steeringWheelDeg), Math.abs(right.steeringWheelDeg)) < 0.03, 'left/right steering-wheel response is asymmetric');
  assert(rel(Math.abs((left.flDeg + left.frDeg) * 0.5), Math.abs((right.flDeg + right.frDeg) * 0.5)) < 0.04, 'left/right road-wheel response is asymmetric');
  assert(rel(Math.abs(left.yawRateDegS), Math.abs(right.yawRateDegS)) < 0.08, 'left/right yaw response is asymmetric');
}

// Steering while stationary may load the contact patch but must not inject a yaw assist.
{
  const sim = makeSim(0);
  const rows = run(sim, 0.8, () => 0.5);
  assert(Math.max(...rows.map((row) => Math.abs(row.speedKmh))) < 1.0, 'stationary steering injected vehicle speed');
  assert(Math.max(...rows.map((row) => Math.abs(row.yawRateDegS))) < 3.0, 'stationary steering injected artificial yaw');
}

console.log('SteeringDynamicsTests: PASS');
