import fs from 'node:fs';
import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES, BMW_M5_2025_TARGETS } from '../src/physics/m5G90';

const DT = 1 / 120;
const MPH_PER_MS = 2.2369362921;
const G = 9.81;
const cfg: any = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES };
const zero: any = { throttle: 0, brake: 0, steer: 0, handbrake: false, shiftUp: false, shiftDown: false };

function prepAtSpeed(mph: number) {
  const sim = new Simulation(cfg);
  sim.reset(0, 0, 0);
  const v = mph / MPH_PER_MS;
  sim.vehicle.rigidBody.velocity = { x: 0, y: 0, z: v };
  sim.vehicle.wheels.forEach((w: any) => w.reset(v));
  sim.vehicle.powertrain.gear = 0;
  for (let i = 0; i < 60; i++) sim.stepExplicit(zero, 1);
  return sim;
}

function acceleration(useLaunchPreload: boolean) {
  const sim = new Simulation(cfg);
  sim.reset(0, 0, 0);
  sim.vehicle.powertrain.isAutomatic = true;

  if (useLaunchPreload) {
    // Instrumented M5 acceleration uses launch control: brake held, powertrain
    // loaded, then brake released. Do not count staging time in the run.
    for (let i = 0; i < 120 * 1.5; i++) {
      sim.stepExplicit({ ...zero, throttle: 1, brake: 1 }, 1);
    }
  }

  const launchRpm = sim.vehicle.getState().rpm;
  const launchBoostPsi = sim.vehicle.getState().turboBoostPsi;
  let t60: number | null = null;
  let t100: number | null = null;
  let qTime: number | null = null;
  let qTrap: number | null = null;
  const z0 = sim.vehicle.getState().z;
  for (let i = 0; i < 120 * 18; i++) {
    const st = sim.stepExplicit({ ...zero, throttle: 1 }, 1);
    const t = (i + 1) * DT;
    const mph = st.speedMs * MPH_PER_MS;
    const dist = Math.abs(st.z - z0);
    if (t60 === null && mph >= 60) t60 = t;
    if (t100 === null && mph >= 100) t100 = t;
    if (qTime === null && dist >= 402.336) { qTime = t; qTrap = mph; }
    if (st.rpm > 6800 && sim.vehicle.powertrain.gear < 8) sim.vehicle.powertrain.shiftUp();
    if (qTime !== null && t100 !== null) break;
  }
  return { useLaunchPreload, launchRpm, launchBoostPsi, zeroTo60: t60, zeroTo100: t100, quarterMile: qTime, quarterTrapMph: qTrap };
}

function braking(startMph: number, mode: 'OFF'|'SPORT'|'FULL', pedal: number) {
  const sim = prepAtSpeed(startMph);
  sim.vehicle.driverAids.config.absMode = mode as any;
  const z0 = sim.vehicle.getState().z;
  const samples: number[] = [];
  const slips: number[] = [];
  const pressures: number[] = [];
  let peakG = 0;
  for (let i = 0; i < 120 * 12; i++) {
    const st = sim.stepExplicit({ ...zero, brake: pedal }, 1);
    const decel = Math.abs(sim.vehicle.rigidBody.acceleration.z / G);
    peakG = Math.max(peakG, decel);
    if (st.speedMs > 2) {
      samples.push(decel);
      slips.push(...st.wheels.map((w: any) => Math.abs(w.slipRatio)));
      pressures.push(...sim.vehicle.brakes.pressureModulators);
    }
    if (st.speedMs < 0.8) break;
  }
  const distanceM = Math.abs(sim.vehicle.getState().z - z0);
  const avg = (xs: number[]) => xs.reduce((a, b) => a + b, 0) / Math.max(1, xs.length);
  return {
    startMph, mode, pedal,
    distanceM,
    distanceFt: distanceM / 0.3048,
    meanG: avg(samples),
    peakG,
    meanAbsSlip: avg(slips),
    peakAbsSlip: Math.max(...slips),
    meanPressure: avg(pressures),
    minPressure: Math.min(...pressures),
  };
}

function skidpad() {
  const sweeps: any[] = [];
  for (const steer of [0.06,0.08,0.10,0.12,0.14,0.16,0.18,0.20,0.22]) {
    const sim = prepAtSpeed(48);
    const rows: any[] = [];
    for (let i = 0; i < 120 * 3; i++) {
      const st = sim.stepExplicit({ ...zero, steer }, 1);
      if (i > 240) {
        rows.push({
          g: Math.abs(sim.vehicle.rigidBody.acceleration.x / G),
          speed: st.speedMs * MPH_PER_MS,
          frontSlip: (Math.abs(st.wheels[0].slipAngle) + Math.abs(st.wheels[1].slipAngle)) * 0.5,
          rearSlip: (Math.abs(st.wheels[2].slipAngle) + Math.abs(st.wheels[3].slipAngle)) * 0.5,
        });
      }
    }
    const avg = (k: string) => rows.reduce((a, r) => a + r[k], 0) / rows.length;
    sweeps.push({
      steer,
      latG: avg('g'),
      speedMph: avg('speed'),
      frontSlipDeg: avg('frontSlip') * 180 / Math.PI,
      rearSlipDeg: avg('rearSlip') * 180 / Math.PI,
    });
  }
  return { sweeps, peakG: Math.max(...sweeps.map(x => x.latG)) };
}

function staticLoads() {
  const sim = new Simulation(cfg);
  sim.reset(0, 0, 0);
  for (let i = 0; i < 360; i++) sim.stepExplicit(zero, 1);
  const s = sim.vehicle.getState();
  const loads = s.wheels.map((w: any) => w.suspensionForce);
  const front = loads[0] + loads[1];
  const total = loads.reduce((a: number, b: number) => a + b, 0);
  return { loads, frontFraction: front / total, bodyY: s.y };
}

const brakeSweep70 = [
  braking(70,'SPORT',1.0), braking(70,'SPORT',0.8), braking(70,'SPORT',0.65),
  braking(70,'FULL',1.0), braking(70,'OFF',0.55), braking(70,'OFF',0.70),
];
const brakeSweep100 = [
  braking(100,'SPORT',1.0), braking(100,'SPORT',0.8), braking(100,'SPORT',0.65),
  braking(100,'FULL',1.0), braking(100,'OFF',0.55), braking(100,'OFF',0.70),
];

const result = {
  targets: BMW_M5_2025_TARGETS,
  config: {
    massKg: cfg.mass,
    frontWeight: cfg.weightDistributionFront,
    wheelbaseM: cfg.wheelbase,
    trackM: cfg.trackWidth,
    wheelRadiusM: cfg.wheelRadius,
    gears: cfg.forwardGearRatios,
    finalDrive: cfg.finalDriveRatio,
  },
  staticLoads: staticLoads(),
  accelerationLaunchControl: acceleration(true),
  accelerationIdle: acceleration(false),
  brakeSweep70,
  brakeSweep100,
  skidpad: skidpad(),
};

fs.writeFileSync('m5-calibration.json', JSON.stringify(result, null, 2));
console.log(JSON.stringify(result, null, 2));
