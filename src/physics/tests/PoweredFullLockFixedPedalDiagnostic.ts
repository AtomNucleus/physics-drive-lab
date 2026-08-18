import assert from 'node:assert/strict';
import { mkdirSync, writeFileSync } from 'node:fs';
import type { ControlInputs, VehicleConfig } from '../../types';
import { Simulation } from '../Simulation';
import { ProvingGroundSurfaceProvider } from '../SurfaceProvider';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { PhysicsMath } from '../math/PhysicsMath';

const DEG = 180 / Math.PI;
const TARGET_SPEED_MS = 10 / 3.6;
const baseConfig = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES } as VehicleConfig;
const inputsBase: ControlInputs = {
  throttle: 0,
  brake: 0,
  steer: 1,
  handbrake: false,
  shiftUp: false,
  shiftDown: false,
};

const range = (v: number[]) => Math.max(...v) - Math.min(...v);

function makeSim(overrides: Record<string, unknown> = {}) {
  const config = { ...baseConfig, ...overrides } as VehicleConfig;
  const sim = new Simulation(config, new ProvingGroundSurfaceProvider());
  sim.reset(0, 0, 0);
  sim.vehicle.powertrain.isAutomatic = false;
  sim.vehicle.powertrain.gear = 0;
  for (let i = 0; i < 360; i++) sim.stepExplicit({ ...inputsBase, steer: 0 }, 1);
  sim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, TARGET_SPEED_MS);
  for (const wheel of sim.vehicle.wheels) wheel.reset(TARGET_SPEED_MS);
  sim.vehicle.powertrain.isAutomatic = true;
  sim.vehicle.powertrain.gear = 1;
  sim.vehicle.powertrain.engineRpm = config.idleRpm;
  sim.vehicle.powertrain.flywheelRpm = config.idleRpm;
  return sim;
}

function run(throttle: number, hz = 120, overrides: Record<string, unknown> = {}) {
  const sim = makeSim(overrides);
  sim.fixedDt = 1 / hz;
  const totalSteps = hz * 6;
  const steadyStart = hz * 2;
  const speed: number[] = [];
  const roll: number[] = [];
  const latG: number[] = [];
  const yaw: number[] = [];
  const travel = [[], [], [], []] as number[][];
  const loads = [[], [], [], []] as number[][];
  const slips = [[], [], [], []] as number[][];
  const forceFlips = [0, 0, 0, 0];
  const prevSigns = [0, 0, 0, 0];
  let tcsSamples = 0;

  for (let step = 0; step < totalSteps; step++) {
    const state = sim.stepExplicit({ ...inputsBase, throttle }, 1);
    if (state.tcsActive) tcsSamples++;
    for (let i = 0; i < 4; i++) {
      const f = state.wheels[i].forceVectorLong;
      const sign = Math.abs(f) > 80 ? Math.sign(f) : 0;
      if (sign && prevSigns[i] && sign !== prevSigns[i]) forceFlips[i]++;
      if (sign) prevSigns[i] = sign;
    }
    if (step < steadyStart) continue;
    speed.push(state.speedKmh);
    roll.push(state.roll * DEG);
    latG.push(state.lateralG);
    yaw.push(state.yawRate * DEG);
    for (let i = 0; i < 4; i++) {
      travel[i].push(state.wheels[i].verticalTravelM * 1000);
      loads[i].push(state.wheels[i].forceVectorNorm);
      slips[i].push(state.wheels[i].slipRatio);
    }
  }

  const result = {
    throttle,
    hz,
    overrides,
    speedKmh: { min: Math.min(...speed), max: Math.max(...speed), p2p: range(speed) },
    rollP2pDeg: range(roll),
    lateralGP2p: range(latG),
    yawRateP2pDegS: range(yaw),
    wheelTravelP2pMm: travel.map(range),
    tireLoadP2pN: loads.map(range),
    slipRatioP2p: slips.map(range),
    longForceSignFlips: forceFlips,
    tcsSamples,
  };
  assert(Number.isFinite(result.rollP2pDeg));
  return result;
}

const relaxationVariants = [
  { label: 'baseline', overrides: {} },
  {
    label: 'medium-fast',
    overrides: { longitudinalRelaxationLength: 0.060, longitudinalForceRelaxationLength: 0.030 },
  },
  {
    label: 'minimum-clamped',
    overrides: { longitudinalRelaxationLength: 0.025, longitudinalForceRelaxationLength: 0.018 },
  },
].map(({ label, overrides }) => ({ label, result: run(0.05, 120, overrides) }));

const inertiaVariants = [
  { label: 'baseline', overrides: {} },
  { label: 'double-driveline-coupling', overrides: { drivelineInertiaCoupling: 2.0 } },
  { label: 'double-wheel-inertia', overrides: { wheelInertia: Number(baseConfig.wheelInertia) * 2 } },
].map(({ label, overrides }) => ({ label, result: run(0.05, 120, overrides) }));

const result = {
  scenario: 'M5 10 km/h full-lock fixed-pedal solver isolation',
  fixedPedal: [0.02, 0.03, 0.05, 0.08, 0.10].map((t) => run(t)),
  timestepAB: [120, 240, 480].map((hz) => run(0.05, hz)),
  relaxationVariants,
  inertiaVariants,
};

mkdirSync('artifacts', { recursive: true });
writeFileSync('artifacts/powered-full-lock-fixed-pedal.json', `${JSON.stringify(result, null, 2)}\n`);
console.log(JSON.stringify(result, null, 2));
console.log('PoweredFullLockFixedPedalDiagnostic: PASS');
