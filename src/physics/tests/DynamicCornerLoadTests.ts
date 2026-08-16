import { Simulation } from '../Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { PhysicsMath } from '../math/PhysicsMath';

const assert = (condition: boolean, message: string) => {
  if (!condition) throw new Error(message);
};

const config = {
  ...DEFAULT_VEHICLE_CONFIG,
  ...BMW_M5_2025_OVERRIDES,
} as any;

const sim = new Simulation(config);
sim.reset(0, 0, 0);

const neutral = {
  throttle: 0,
  brake: 0,
  steer: 0,
  handbrake: false,
  shiftUp: false,
  shiftDown: false,
};

// Settle the sprung/unsprung system before introducing road speed.
for (let i = 0; i < 240; i++) sim.stepExplicit(neutral, 1);

// Give the complete vehicle a clean 90 km/h free-rolling initial condition. This
// isolates steering/load-transfer signs from launch, shifting and TCS behavior.
const speedMs = 25;
sim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, speedMs);
sim.vehicle.wheels.forEach((wheel) => wheel.reset(speedMs));

// Allow tire rotational state and suspension to settle at speed before turn-in.
for (let i = 0; i < 60; i++) sim.stepExplicit(neutral, 1);

let leftLoadSum = 0;
let rightLoadSum = 0;
let samples = 0;
let peakLoadDeltaN = -Infinity;
let peakRollDeg = 0;
let peakYawRateDegS = 0;

// In the UI, LEFT steering is positive input. DriverAids converts that to the
// simulation's negative steering angle convention. Run a moderate 18% steering
// step, long enough for tire relaxation + suspension load transfer to establish.
for (let step = 0; step < 120; step++) {
  const state = sim.stepExplicit({ ...neutral, steer: 0.18 }, 1);
  const leftLoad = state.wheels[0].suspensionForce + state.wheels[2].suspensionForce;
  const rightLoad = state.wheels[1].suspensionForce + state.wheels[3].suspensionForce;
  const delta = rightLoad - leftLoad;

  peakLoadDeltaN = Math.max(peakLoadDeltaN, delta);
  peakRollDeg = Math.max(peakRollDeg, Math.abs(state.roll * 180 / Math.PI));
  peakYawRateDegS = Math.max(peakYawRateDegS, Math.abs(state.yawRate * 180 / Math.PI));

  // Ignore the first 0.25 s so the assertion measures established load transfer,
  // not the instant steering command before the tire/suspension transients build.
  if (step >= 30) {
    leftLoadSum += leftLoad;
    rightLoadSum += rightLoad;
    samples++;
  }
}

const averageLeftLoadN = leftLoadSum / Math.max(1, samples);
const averageRightLoadN = rightLoadSum / Math.max(1, samples);
const averageOutsideLoadGainN = averageRightLoadN - averageLeftLoadN;

assert(peakYawRateDegS > 2, 'left steering command did not generate a meaningful turn');
assert(peakRollDeg > 0.05, 'left steering command did not generate measurable chassis roll');
assert(
  averageRightLoadN > averageLeftLoadN,
  `LEFT turn must load RIGHT/outside tires: left=${averageLeftLoadN.toFixed(0)}N right=${averageRightLoadN.toFixed(0)}N`
);
assert(
  averageOutsideLoadGainN > 500,
  `outside-load transfer is too small to be physical at 90 km/h: ${averageOutsideLoadGainN.toFixed(0)}N`
);

console.log(JSON.stringify({
  test: 'dynamic left-turn outside-load transfer',
  input: 'left steer +0.18 at 90 km/h',
  averageLeftLoadN,
  averageRightLoadN,
  averageOutsideLoadGainN,
  peakOutsideLoadDeltaN: peakLoadDeltaN,
  peakRollDeg,
  peakYawRateDegS,
  expected: 'FR + RR > FL + RL',
  status: 'passed',
}, null, 2));
