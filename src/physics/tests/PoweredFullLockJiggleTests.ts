import assert from 'node:assert/strict';
import type { ControlInputs, VehicleConfig } from '../../types';
import { Simulation } from '../Simulation';
import { ProvingGroundSurfaceProvider } from '../SurfaceProvider';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { PhysicsMath } from '../math/PhysicsMath';

const DT = 1 / 120;
const TARGET_SPEED_MS = 10 / 3.6;
const DEG = 180 / Math.PI;
const config = {
  ...DEFAULT_VEHICLE_CONFIG,
  ...BMW_M5_2025_OVERRIDES,
} as VehicleConfig;

const baseInputs: ControlInputs = {
  throttle: 0,
  brake: 0,
  steer: 1,
  handbrake: false,
  shiftUp: false,
  shiftDown: false,
};

function makeAtTenKmh(automaticDrive: boolean) {
  const sim = new Simulation(config, new ProvingGroundSurfaceProvider());
  sim.reset(0, 0, 0);

  sim.vehicle.powertrain.isAutomatic = false;
  sim.vehicle.powertrain.gear = 0;
  for (let i = 0; i < 360; i++) sim.stepExplicit({ ...baseInputs, steer: 0 }, 1);

  sim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, TARGET_SPEED_MS);
  for (const wheel of sim.vehicle.wheels) wheel.reset(TARGET_SPEED_MS);

  sim.vehicle.powertrain.isAutomatic = automaticDrive;
  sim.vehicle.powertrain.gear = automaticDrive ? 1 : 0;
  sim.vehicle.powertrain.engineRpm = config.idleRpm;
  sim.vehicle.powertrain.flywheelRpm = config.idleRpm;
  return sim;
}

const range = (values: number[]) => Math.max(...values) - Math.min(...values);
const minMax = (values: number[]) => ({
  min: Math.min(...values),
  max: Math.max(...values),
  mean: values.reduce((sum, value) => sum + value, 0) / Math.max(1, values.length),
});
const rms = (values: number[]) =>
  Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / Math.max(1, values.length));

function secondDifferenceRms(values: number[]) {
  if (values.length < 3) return 0;
  const diffs: number[] = [];
  for (let i = 2; i < values.length; i++) {
    diffs.push(values[i] - 2 * values[i - 1] + values[i - 2]);
  }
  return rms(diffs);
}

function highPassRms(values: number[], halfWindow = 8) {
  if (values.length < halfWindow * 2 + 1) return 0;
  const residuals: number[] = [];
  for (let i = halfWindow; i < values.length - halfWindow; i++) {
    let sum = 0;
    for (let j = i - halfWindow; j <= i + halfWindow; j++) sum += values[j];
    const mean = sum / (halfWindow * 2 + 1);
    residuals.push(values[i] - mean);
  }
  return rms(residuals);
}

type ScenarioMode = 'neutral-coast' | 'automatic-creep' | 'automatic-speed-hold';
type ScenarioOptions = {
  tcsOff?: boolean;
  openDifferential?: boolean;
};

function runScenario(mode: ScenarioMode, options: ScenarioOptions = {}) {
  const automaticDrive = mode !== 'neutral-coast';
  const sim = makeAtTenKmh(automaticDrive);
  if (options.tcsOff) sim.vehicle.driverAids.config.tcsMode = 'OFF';
  if (options.openDifferential) sim.vehicle.differential.config.type = 'OPEN';

  const totalSteps = 120 * 6;
  const steadyStart = 120 * 2;

  const speed: number[] = [];
  const roll: number[] = [];
  const heave: number[] = [];
  const wheelTravel = [[], [], [], []] as number[][];
  const hubY = [[], [], [], []] as number[][];
  const tireLoad = [[], [], [], []] as number[][];
  const slipRatio = [[], [], [], []] as number[][];
  const omega = [[], [], [], []] as number[][];
  const driveTorque = [[], [], [], []] as number[][];
  const driveForceDemand = [[], [], [], []] as number[][];
  const adhesionCapacity = [[], [], [], []] as number[][];
  const adhesionAuthority = [[], [], [], []] as number[][];
  const adhesionSpeedAuthority = [[], [], [], []] as number[][];
  const adhesionTorqueAuthority = [[], [], [], []] as number[][];
  const longForceSignFlips = [0, 0, 0, 0];
  const previousLongForceSign = [0, 0, 0, 0];
  const airborneToggles = [0, 0, 0, 0];
  const previousAirborne = [false, false, false, false];
  let throttleMin = Infinity;
  let throttleMax = -Infinity;
  let tcsSamples = 0;

  for (let step = 0; step < totalSteps; step++) {
    const currentSpeedMs = Math.abs(sim.vehicle.rigidBody.getLocalVelocity().z);
    let throttle = 0;
    if (mode === 'automatic-speed-hold') {
      const error = TARGET_SPEED_MS - currentSpeedMs;
      throttle = PhysicsMath.clamp(0.055 + error * 0.055, 0, 0.18);
    }
    throttleMin = Math.min(throttleMin, throttle);
    throttleMax = Math.max(throttleMax, throttle);

    const state = sim.stepExplicit({ ...baseInputs, throttle }, 1);
    if (state.tcsActive) tcsSamples++;

    for (let i = 0; i < 4; i++) {
      const force = state.wheels[i].forceVectorLong;
      const sign = Math.abs(force) > 80 ? Math.sign(force) : 0;
      if (sign !== 0 && previousLongForceSign[i] !== 0 && sign !== previousLongForceSign[i]) {
        longForceSignFlips[i]++;
      }
      if (sign !== 0) previousLongForceSign[i] = sign;

      const airborne = state.wheels[i].isAirborne;
      if (step > 0 && airborne !== previousAirborne[i]) airborneToggles[i]++;
      previousAirborne[i] = airborne;
    }

    if (step < steadyStart) continue;

    speed.push(state.speedKmh);
    roll.push(state.roll * DEG);
    heave.push(state.heave * 1000);
    for (let i = 0; i < 4; i++) {
      const wheelDynamics = sim.vehicle.wheels[i];
      wheelTravel[i].push(state.wheels[i].verticalTravelM * 1000);
      hubY[i].push(state.wheels[i].hubWorldPos.y * 1000);
      tireLoad[i].push(state.wheels[i].forceVectorNorm);
      slipRatio[i].push(state.wheels[i].slipRatio);
      omega[i].push(state.wheels[i].angularVelocity);
      driveTorque[i].push(wheelDynamics.driveTorqueNm);
      driveForceDemand[i].push(wheelDynamics.driveForceDemandN);
      adhesionCapacity[i].push(wheelDynamics.longitudinalAdhesionCapacityN);
      adhesionAuthority[i].push(wheelDynamics.driveAdhesionAuthority);
      adhesionSpeedAuthority[i].push(wheelDynamics.driveAdhesionSpeedAuthority);
      adhesionTorqueAuthority[i].push(wheelDynamics.driveAdhesionTorqueAuthority);
    }
  }

  const result = {
    mode,
    options,
    gear: sim.vehicle.powertrain.gear,
    differentialType: sim.vehicle.differential.config.type,
    tcsMode: sim.vehicle.driverAids.config.tcsMode,
    tcsSamples,
    throttleRange: [throttleMin, throttleMax],
    speedKmh: { min: Math.min(...speed), max: Math.max(...speed), p2p: range(speed) },
    roll: {
      p2pDeg: range(roll),
      highPassRmsDeg: highPassRms(roll),
      secondDiffRmsDeg: secondDifferenceRms(roll),
    },
    heave: {
      p2pMm: range(heave),
      highPassRmsMm: highPassRms(heave),
      secondDiffRmsMm: secondDifferenceRms(heave),
    },
    wheelTravelP2pMm: wheelTravel.map(range),
    wheelTravelHighPassRmsMm: wheelTravel.map((v) => highPassRms(v)),
    hubYHighPassRmsMm: hubY.map((v) => highPassRms(v)),
    tireLoadP2pN: tireLoad.map(range),
    tireLoadHighPassRmsN: tireLoad.map((v) => highPassRms(v)),
    slipRatioP2p: slipRatio.map(range),
    omegaSecondDiffRms: omega.map((v) => secondDifferenceRms(v)),
    solverTelemetry: {
      driveTorqueNm: driveTorque.map(minMax),
      driveForceDemandN: driveForceDemand.map(minMax),
      longitudinalAdhesionCapacityN: adhesionCapacity.map(minMax),
      adhesionAuthority: adhesionAuthority.map(minMax),
      adhesionSpeedAuthority: adhesionSpeedAuthority.map(minMax),
      adhesionTorqueAuthority: adhesionTorqueAuthority.map(minMax),
    },
    longForceSignFlips,
    airborneToggles,
  };

  assert(speed.every(Number.isFinite), `${mode}: non-finite speed`);
  assert(roll.every(Number.isFinite), `${mode}: non-finite roll`);
  assert(airborneToggles.every((count) => count === 0), `${mode}: wheel contact toggled ${airborneToggles.join(',')}`);
  return result;
}

const neutral = runScenario('neutral-coast');
const creep = runScenario('automatic-creep');
const powered = runScenario('automatic-speed-hold');
const poweredTcsOff = runScenario('automatic-speed-hold', { tcsOff: true });
const poweredOpenDiff = runScenario('automatic-speed-hold', { openDifferential: true });
const poweredOpenDiffTcsOff = runScenario('automatic-speed-hold', {
  openDifferential: true,
  tcsOff: true,
});

console.log(JSON.stringify({
  scenario: '2025 M5 10 km/h full-lock live-drivetrain jiggle isolation diagnostic',
  neutral,
  creep,
  powered,
  poweredTcsOff,
  poweredOpenDiff,
  poweredOpenDiffTcsOff,
}, null, 2));

console.log('PoweredFullLockJiggleTests: ISOLATION DIAGNOSTIC PASS');
