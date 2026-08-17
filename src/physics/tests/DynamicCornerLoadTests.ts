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

const neutral = {
  throttle: 0,
  brake: 0,
  steer: 0,
  handbrake: false,
  shiftUp: false,
  shiftDown: false,
};

// ---------------------------------------------------------------------------
// Full-vehicle outside-corner load transfer
// ---------------------------------------------------------------------------
const sim = new Simulation(config);
sim.reset(0, 0, 0);

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

// LEFT steering is positive input. In the canonical right-handed vehicle frame,
// +X is vehicle-left and +Z is forward. Run a moderate 18% steering step long
// enough for tire relaxation + suspension load transfer to establish.
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

// ---------------------------------------------------------------------------
// High-speed flat-road sprung/unsprung stability
// ---------------------------------------------------------------------------
const highSpeedSim = new Simulation(config);
highSpeedSim.reset(0, 0, 0);
for (let i = 0; i < 360; i++) highSpeedSim.stepExplicit(neutral, 1);

const highSpeedMs = 250 / 3.6;
highSpeedSim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, highSpeedMs);
highSpeedSim.vehicle.wheels.forEach((wheel) => wheel.reset(highSpeedMs));

const highSpeedInitial = highSpeedSim.vehicle.getState();
const baselineHeave = highSpeedInitial.heave;
let maxHeaveDeltaM = 0;
let maxVerticalGMagnitude = 0;
let maxTravelM = -Infinity;
let minTravelM = Infinity;
let airborneSamples = 0;
let nonFiniteSamples = 0;
let maxYawDeviationDeg = 0;
const initialYaw = highSpeedInitial.yaw;

// Five seconds on the literal flat proving-ground plane. With no bump input, a
// stable unsprung model must not excite wheel hop on its own or let the chassis
// visually/physically separate from the four wheel centers at high speed.
for (let step = 0; step < 600; step++) {
  const state = highSpeedSim.stepExplicit(neutral, 1);
  maxHeaveDeltaM = Math.max(maxHeaveDeltaM, Math.abs(state.heave - baselineHeave));
  maxVerticalGMagnitude = Math.max(maxVerticalGMagnitude, Math.abs(state.verticalG));
  maxYawDeviationDeg = Math.max(
    maxYawDeviationDeg,
    Math.abs((state.yaw - initialYaw) * 180 / Math.PI)
  );

  for (const wheel of state.wheels) {
    maxTravelM = Math.max(maxTravelM, wheel.verticalTravelM);
    minTravelM = Math.min(minTravelM, wheel.verticalTravelM);
    if (wheel.isAirborne) airborneSamples++;
    if (
      !Number.isFinite(wheel.verticalTravelM) ||
      !Number.isFinite(wheel.suspensionForce) ||
      !Number.isFinite(wheel.forceVectorNorm)
    ) {
      nonFiniteSamples++;
    }
  }
}

assert(nonFiniteSamples === 0, 'high-speed suspension produced non-finite wheel state');
assert(airborneSamples === 0, `flat road generated ${airborneSamples} false airborne wheel samples`);
assert(maxTravelM <= 0.140001, `suspension exceeded max bump travel at speed: ${maxTravelM} m`);
assert(minTravelM >= -0.120001, `suspension exceeded max droop travel at speed: ${minTravelM} m`);
assert(
  maxHeaveDeltaM < 0.02,
  `chassis separated vertically from settled ride height on flat road: ${maxHeaveDeltaM} m`
);
assert(
  maxYawDeviationDeg < 0.1,
  `straight 250 km/h flat-road run developed spurious yaw: ${maxYawDeviationDeg} deg`
);

// ---------------------------------------------------------------------------
// Full-vehicle road -> wheel -> suspension -> chassis force-path regression
// ---------------------------------------------------------------------------
let bumpRoadElevation = 0;
const bumpSurface = {
  sampleSurface: () => ({
    elevation: bumpRoadElevation,
    normal: PhysicsMath.vec3(0, 1, 0),
    slopePitch: 0,
    slopeRoll: 0,
    type: 'asphalt' as const,
    friction: 1.0,
    rollingResistance: 0.015,
    wetness: 0,
    isKerbRumble: false,
  }),
};

const forcePathSim = new Simulation(config, bumpSurface);
forcePathSim.reset(0, 0, 0);
for (let i = 0; i < 720; i++) forcePathSim.stepExplicit(neutral, 1);

// A sudden 20 mm road step produces an immediate tire-load spike because the tire
// deflects against the unsprung wheel. The chassis must NOT receive that same spike
// directly. It should only receive the spring/damper/ARB/hard-stop reaction that the
// wheel has transmitted through the suspension during this 120 Hz step.
bumpRoadElevation = 0.020;
forcePathSim.vehicle.step(neutral, forcePathSim.fixedDt);

const forcePathTireLoadN = forcePathSim.vehicle.suspension.states.reduce(
  (sum, state) => sum + state.tireNormalForceN,
  0
);
const forcePathChassisLoadN = forcePathSim.vehicle.suspension.states.reduce(
  (sum, state) => sum + state.chassisForceN,
  0
);
const actualVerticalAccelerationMps2 = forcePathSim.vehicle.rigidBody.acceleration.y;
const expectedFromSuspensionMps2 = forcePathChassisLoadN / config.mass - 9.81;
const legacyDirectTireLoadMps2 = forcePathTireLoadN / config.mass - 9.81;
const forcePathLoadSeparationN = Math.abs(forcePathTireLoadN - forcePathChassisLoadN);

assert(
  forcePathLoadSeparationN > 1000,
  `road step did not create enough tire-vs-chassis load separation to test coupling: ${forcePathLoadSeparationN.toFixed(0)}N`
);
assert(
  Math.abs(actualVerticalAccelerationMps2 - expectedFromSuspensionMps2) < 0.08,
  `chassis acceleration must follow suspension reaction, not contact-patch load: actual=${actualVerticalAccelerationMps2.toFixed(3)} expected=${expectedFromSuspensionMps2.toFixed(3)} m/s^2`
);
assert(
  Math.abs(actualVerticalAccelerationMps2 - legacyDirectTireLoadMps2) > 0.20,
  'vehicle still appears to bypass the suspension by applying tire normal load directly to the chassis'
);

console.log(JSON.stringify({
  leftTurn90Kmh: {
    averageLeftLoadN,
    averageRightLoadN,
    averageOutsideLoadGainN,
    peakOutsideLoadDeltaN: peakLoadDeltaN,
    peakRollDeg,
    peakYawRateDegS,
    expected: 'FR + RR > FL + RL',
  },
  flatRoad250Kmh: {
    durationSec: 5,
    maxHeaveDeltaM,
    maxVerticalGMagnitude,
    minTravelM,
    maxTravelM,
    airborneSamples,
    maxYawDeviationDeg,
  },
  suspensionForcePath: {
    roadStepM: bumpRoadElevation,
    tireNormalLoadN: forcePathTireLoadN,
    chassisReactionLoadN: forcePathChassisLoadN,
    loadSeparationN: forcePathLoadSeparationN,
    actualVerticalAccelerationMps2,
    expectedFromSuspensionMps2,
    legacyDirectTireLoadMps2,
    expected: 'road -> tire/unsprung -> spring/damper -> chassis',
  },
  status: 'passed',
}, null, 2));