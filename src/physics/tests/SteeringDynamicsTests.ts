import { Simulation } from '../Simulation';
import { TireModel } from '../TireModel';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { PhysicsMath } from '../math/PhysicsMath';

const assert = (condition: boolean, message: string) => {
  if (!condition) throw new Error(message);
};
const DEG = 180 / Math.PI;
const config = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES } as any;
const neutral = { throttle: 0, brake: 0, steer: 0, handbrake: false, shiftUp: false, shiftDown: false };
const maxAbs = (values: number[]) => Math.max(0, ...values.map(Math.abs));
const mean = (values: number[]) => values.reduce((a, b) => a + b, 0) / Math.max(1, values.length);

type Sample = {
  t: number; speedKmh: number; steeringWheelDeg: number; rackMm: number; rackCenterDeg: number;
  rackRateDegS: number; leftRoadWheelDeg: number; rightRoadWheelDeg: number; alphaFLDeg: number;
  alphaFRDeg: number; frontFyN: number; fzFLN: number; fzFRN: number; tireSatNm: number;
  mechanicalTrailNm: number; epsNm: number; rackTorqueNm: number; ffbNm: number; yawRateDegS: number;
  lateralG: number; rollDeg: number; leftComplianceDeg: number; rightComplianceDeg: number; epsGain: number;
};

const sample = (sim: Simulation, t: number): Sample => {
  const state = sim.vehicle.getState();
  const s = sim.suspensionKinematics.steeringDynamics.telemetry;
  const out: Sample = {
    t,
    speedKmh: state.speedKmh,
    steeringWheelDeg: s.steeringWheelAngleRad * DEG,
    rackMm: s.rackDisplacementM * 1000,
    rackCenterDeg: s.rackCenterAngleRad * DEG,
    rackRateDegS: s.rackAngularVelocityRadS * DEG,
    leftRoadWheelDeg: state.wheels[0].steerAngle * DEG,
    rightRoadWheelDeg: state.wheels[1].steerAngle * DEG,
    alphaFLDeg: state.wheels[0].slipAngle * DEG,
    alphaFRDeg: state.wheels[1].slipAngle * DEG,
    frontFyN: state.wheels[0].forceVectorLat + state.wheels[1].forceVectorLat,
    fzFLN: state.wheels[0].forceVectorNorm,
    fzFRN: state.wheels[1].forceVectorNorm,
    tireSatNm: s.torques.tireSelfAligningRoadNm,
    mechanicalTrailNm: s.torques.casterMechanicalTrailRoadNm,
    epsNm: s.torques.epsAssistSteeringWheelNm,
    rackTorqueNm: s.torques.netRackRoadNm,
    ffbNm: s.torques.ffbReadySteeringWheelNm,
    yawRateDegS: state.yawRate * DEG,
    lateralG: state.lateralG,
    rollDeg: state.roll * DEG,
    leftComplianceDeg: s.leftComplianceRad * DEG,
    rightComplianceDeg: s.rightComplianceRad * DEG,
    epsGain: s.epsAssistGain,
  };
  Object.entries(out).forEach(([key, value]) => assert(Number.isFinite(value), `${key} non-finite: ${value}`));
  return out;
};

const createSim = (speedKmh = 0) => {
  const sim = new Simulation(config);
  sim.reset(0, 0, 0);
  for (let i = 0; i < 240; i++) sim.stepExplicit({ ...neutral, brake: speedKmh === 0 ? 0.35 : 0 }, 1);
  if (speedKmh > 0) {
    const speedMs = speedKmh / 3.6;
    sim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, speedMs);
    sim.vehicle.wheels.forEach((wheel) => wheel.reset(speedMs));
    for (let i = 0; i < 24; i++) sim.stepExplicit(neutral, 1);
  }
  return sim;
};

const run = (sim: Simulation, duration: number, inputAt: (t: number) => typeof neutral) => {
  const out: Sample[] = [];
  const steps = Math.ceil(duration / sim.fixedDt);
  for (let i = 0; i < steps; i++) {
    sim.stepExplicit(inputAt(i * sim.fixedDt), 1);
    out.push(sample(sim, (i + 1) * sim.fixedDt));
  }
  return out;
};

// Published geometry/ratio invariants and Ackermann symmetry.
{
  const steering = createSim().suspensionKinematics.steeringDynamics;
  const turns = steering.maxSteeringWheelAngleRad() / Math.PI;
  const l = steering.ackermannForCenter(0.25);
  const r = steering.ackermannForCenter(-0.25);
  assert(Math.abs(steering.steeringRatioAt(0) - 14.2) < 1e-9, 'center ratio must be 14.2:1');
  assert(turns > 2.35 && turns < 2.75, `expected ~2.5 turns lock-to-lock, got ${turns.toFixed(2)}`);
  assert(l.left > l.right && l.right > 0, 'left Ackermann inner/outer ordering failed');
  assert(Math.abs(r.right) > Math.abs(r.left) && r.left < 0, 'right Ackermann inner/outer ordering failed');
  assert(Math.abs(l.left + r.right) < 1e-10 && Math.abs(l.right + r.left) < 1e-10, 'Ackermann mirror failed');
}

// Tire SAT must build, peak/fall, and weaken under combined slip.
{
  const tire = new TireModel({
    baseGrip: 1.21, stiffnessB: 15, loadSensitivity: 0.000030, slideFrictionMultiplier: 0.83,
    relaxationLength: 0.50, longitudinalRelaxationLength: 0.12, longitudinalForceRelaxationLength: 0.066,
    pneumaticTrailMax: 0.030, camberStiffness: 85, optimalTemp: 75, basePressurePsi: 35, referenceLoadN: 6000,
  });
  const mz = (alpha: number, kappa = 0) => Math.abs(tire.calculate({
    slipRatio: kappa, slipAngle: alpha, verticalLoad: 6000, camberDeg: 0, surfaceFriction: 1, isLeft: true,
  }).aligningTorque);
  const a02 = mz(0.02), a04 = mz(0.04), a12 = mz(0.12), a25 = mz(0.25), combined = mz(0.04, 0.18);
  assert(a04 > a02, 'SAT did not rise with initial slip');
  assert(a12 < a04 && a25 < a12, 'SAT did not fall as pneumatic trail collapsed');
  assert(combined < a04, 'combined slip did not soften SAT');
}

const stationary = (() => {
  const sim = createSim(0);
  const a = run(sim, 1.2, () => ({ ...neutral, brake: 0.55, steer: 0.75 }));
  const f = a.at(-1)!;
  assert(Math.abs(f.rackCenterDeg) > 12, `stationary EPS did not move rack: ${f.rackCenterDeg.toFixed(2)} deg`);
  assert(maxAbs(a.map((s) => s.speedKmh)) < 1.0, 'stationary steering injected speed');
  assert(maxAbs(a.map((s) => s.yawRateDegS)) < 3.0, 'stationary steering produced violent yaw');
  assert(maxAbs(a.map((s) => s.frontFyN)) < 14000, 'stationary steering produced violent tire force');
  return { final: f, maxFrontFyN: maxAbs(a.map((s) => s.frontFyN)) };
})();

const parking10 = (() => {
  const sim = createSim(10);
  const start = sim.vehicle.getState().speedKmh;
  const a = run(sim, 1.5, () => ({ ...neutral, steer: 0.48 }));
  const f = a.at(-1)!;
  const peak = Math.max(...a.map((s) => s.speedKmh));
  assert(f.leftRoadWheelDeg > f.rightRoadWheelDeg && f.rightRoadWheelDeg > 0, '10 km/h Ackermann failed');
  assert(peak < start + 1.5, `10 km/h turn gained unexplained speed: ${start.toFixed(2)} -> ${peak.toFixed(2)}`);
  assert(maxAbs(a.map((s) => s.rackRateDegS)) <= 3.2 * DEG + 0.2, 'parking rack exceeded rate limit');
  return { start, peak, final: f };
})();

const constantTurn = (speedKmh: number, steer: number, duration = 2.0) => {
  const sim = createSim(speedKmh);
  const a = run(sim, duration, () => ({ ...neutral, steer }));
  const f = a.at(-1)!;
  assert(f.yawRateDegS > 0.2, `${speedKmh} km/h left turn did not build positive yaw`);
  assert(maxAbs(a.map((s) => s.lateralG)) < 1.45, `${speedKmh} km/h lateral acceleration implausible`);
  assert(maxAbs(a.map((s) => s.rollDeg)) < 7.0, `${speedKmh} km/h body roll implausible`);
  assert(maxAbs(a.map((s) => s.leftComplianceDeg)) < 0.40 && maxAbs(a.map((s) => s.rightComplianceDeg)) < 0.40,
    `${speedKmh} km/h compliance visibly floppy`);
  return { sim, a, f };
};

const turn20 = constantTurn(20, 0.22);
const turn30 = constantTurn(30, 0.18);
const turn50 = constantTurn(50, 0.10, 2.4);
assert(Math.abs(mean(turn50.a.slice(-30).map((s) => s.frontFyN))) > 1200, '50 km/h front Fy too small');
assert(maxAbs(turn50.a.map((s) => s.tireSatNm)) > 10, '50 km/h SAT missing');
assert(maxAbs(turn50.a.map((s) => s.mechanicalTrailNm)) > 20, '50 km/h caster/mechanical trail missing');

const lane80 = (() => {
  const sim = createSim(80);
  const a = run(sim, 2.0, (t) => {
    const steer = t < 0.45 ? 0.085 * Math.sin(Math.PI * t / 0.45)
      : t < 0.90 ? -0.085 * Math.sin(Math.PI * (t - 0.45) / 0.45) : 0;
    return { ...neutral, steer };
  });
  assert(maxAbs(a.map((s) => s.rackRateDegS)) <= 3.2 * DEG + 0.2, '80 km/h rack rate exploded');
  assert(maxAbs(a.map((s) => s.yawRateDegS)) < 70, '80 km/h lane change yaw exploded');
  assert(Math.abs(a.at(-1)!.rackCenterDeg) < 2.0, '80 km/h rack failed to settle');
  return a;
})();

const step100 = (() => {
  const sim = createSim(100);
  const a = run(sim, 1.5, (t) => ({ ...neutral, steer: t < 1.0 ? 0.065 : 0 }));
  const first = a[0], at100 = a[Math.round(0.10 / sim.fixedDt) - 1], steady = a[Math.round(0.90 / sim.fixedDt) - 1];
  assert(Math.abs(first.rackCenterDeg) < Math.abs(steady.rackCenterDeg) * 0.45, 'rack angle teleported on first step');
  assert(Math.abs(at100.rackCenterDeg) > Math.abs(first.rackCenterDeg) * 1.4, '100 km/h rack response not progressive');
  assert(maxAbs(a.map((s) => s.yawRateDegS)) < 55, '100 km/h steering step yaw unstable');
  return { first, at100, steady };
})();

const slalom = (() => {
  const sim = createSim(65);
  const a = run(sim, 3.0, (t) => ({ ...neutral, steer: t < 2.0 ? (Math.floor(t / 0.30) % 2 ? -0.11 : 0.11) : 0 }));
  assert(maxAbs(a.map((s) => s.rackRateDegS)) <= 3.2 * DEG + 0.2, 'slalom rack rate exploded');
  assert(maxAbs(a.map((s) => s.rackTorqueNm)) < 4000, 'slalom rack torque exploded');
  assert(Math.abs(a.at(-1)!.rackCenterDeg) < 2.5, 'slalom left rack oscillating');
  return a;
})();

const release = (() => {
  const sim = createSim(50);
  const held = run(sim, 2.0, () => ({ ...neutral, steer: 0.10 }));
  const steady = held.at(-1)!;
  const restoring = steady.tireSatNm + steady.mechanicalTrailNm;
  assert(restoring * steady.rackCenterDeg < 0, `road torque not restoring: ${restoring.toFixed(1)} Nm`);
  const a = run(sim, 1.6, () => neutral);
  const f = a.at(-1)!;
  assert(Math.abs(f.rackCenterDeg) < Math.max(1.0, Math.abs(steady.rackCenterDeg) * 0.25), 'rack refused to self-center');
  assert(Math.abs(f.rollDeg) < Math.max(0.35, Math.abs(steady.rollDeg) * 0.25), 'body failed to settle after release');
  return { steady, final: f };
})();

const brakeSteer = (() => {
  const sim = createSim(80); const start = sim.vehicle.getState().speedKmh;
  const a = run(sim, 1.2, () => ({ ...neutral, steer: 0.075, brake: 0.55 })); const f = a.at(-1)!;
  assert(f.speedKmh < start - 5, 'braking while steering did not reduce speed');
  assert(maxAbs(a.map((s) => s.rackTorqueNm)) < 4500, 'brake/steer destabilized rack');
  return { start, final: f };
})();

const throttleSteer = (() => {
  const sim = createSim(30); const a = run(sim, 1.2, () => ({ ...neutral, steer: 0.11, throttle: 0.32 }));
  assert(maxAbs(a.map((s) => s.yawRateDegS)) < 80, 'throttle/steer yaw exploded');
  assert(maxAbs(a.map((s) => s.rackTorqueNm)) < 4500, 'throttle/steer rack exploded');
  return a.at(-1)!;
})();

const combinedExit = (() => {
  const sim = createSim(50); run(sim, 1.2, () => ({ ...neutral, steer: 0.095 }));
  const a = run(sim, 1.2, () => ({ ...neutral, steer: 0.075, throttle: 0.75 }));
  const state = sim.vehicle.getState();
  const util = Math.max(state.wheels[0].gripUtilization, state.wheels[1].gripUtilization);
  assert(util <= 1.5001, 'combined-slip tire utilization escaped bound');
  assert(maxAbs(a.map((s) => s.rackTorqueNm)) < 5000, 'combined-slip exit destabilized rack');
  return { final: a.at(-1)!, maxFrontUtil: util };
})();

assert(stationary.final.epsGain > step100.steady.epsGain + 3,
  `EPS did not reduce with speed: parking=${stationary.final.epsGain.toFixed(2)} highway=${step100.steady.epsGain.toFixed(2)}`);

console.log(JSON.stringify({
  calibration: {
    wheelbaseM: config.wheelbase, steeringFrontTrackM: config.steeringFrontTrackM,
    ratioCenter: config.steeringRatioCenter, ratioAtLock: config.steeringRatioAtLock,
    maxCenterRoadWheelDeg: config.maxSteerAngle * DEG,
  },
  stationary, parking10, turn20: turn20.f, turn30: turn30.f, turn50: turn50.f,
  lane80: lane80.at(-1), step100, slalom: slalom.at(-1), release, brakeSteer, throttleSteer, combinedExit,
}, null, 2));
console.log('SteeringDynamicsTests: PASS');
