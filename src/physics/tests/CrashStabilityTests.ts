import { Simulation } from '../Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { PhysicsMath } from '../math/PhysicsMath';
import { probeChassisContact } from '../CrashStability';

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

const finiteVehicle = (sim: Simulation) => {
  const body = sim.vehicle.rigidBody;
  return [
    body.position.x, body.position.y, body.position.z,
    body.velocity.x, body.velocity.y, body.velocity.z,
    body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z,
    body.orientation.x, body.orientation.y, body.orientation.z, body.orientation.w,
    ...sim.vehicle.suspension.states.flatMap((state) => [
      state.displacement,
      state.hubPositionWorldY,
      state.hubVelocityWorldY,
      state.tireNormalForceN,
      state.chassisForceN,
    ]),
  ].every(Number.isFinite);
};

// ---------------------------------------------------------------------------
// High-energy upright spin recovery: no solver runaway or suspension separation.
// ---------------------------------------------------------------------------
const spinSim = new Simulation(config);
spinSim.reset(0, 0, 0);
for (let i = 0; i < 360; i++) spinSim.stepExplicit(neutral, 1);

spinSim.vehicle.rigidBody.velocity = PhysicsMath.vec3(13, 0, 29);
spinSim.vehicle.rigidBody.angularVelocity = PhysicsMath.vec3(0.25, 4.2, 0.35);
spinSim.vehicle.rigidBody.orientation = PhysicsMath.quatFromEuler(
  6 * Math.PI / 180,
  0,
  9 * Math.PI / 180
);
spinSim.vehicle.wheels.forEach((wheel) => wheel.reset(29));

let maxSpinAngularSpeed = 0;
let maxSpinHeaveM = 0;
let spinNonFinite = 0;
for (let i = 0; i < 720; i++) {
  const state = spinSim.stepExplicit(neutral, 1);
  const angularSpeed = PhysicsMath.vec3Length(spinSim.vehicle.rigidBody.angularVelocity);
  maxSpinAngularSpeed = Math.max(maxSpinAngularSpeed, angularSpeed);
  maxSpinHeaveM = Math.max(maxSpinHeaveM, Math.abs(state.heave));
  if (!finiteVehicle(spinSim)) spinNonFinite++;
  for (const wheel of state.wheels) {
    assert(wheel.verticalTravelM <= 0.140001, `spin exceeded bump travel: ${wheel.verticalTravelM}`);
    assert(wheel.verticalTravelM >= -0.120001, `spin exceeded droop travel: ${wheel.verticalTravelM}`);
  }
}
assert(spinNonFinite === 0, `spin recovery produced ${spinNonFinite} non-finite samples`);
assert(maxSpinAngularSpeed < 12, `spin angular velocity ran away: ${maxSpinAngularSpeed} rad/s`);
assert(maxSpinHeaveM < 0.45, `spin produced implausible chassis/wheel separation: ${maxSpinHeaveM} m`);

// ---------------------------------------------------------------------------
// Wipeout/roll impact: body shell must not pass through road and excite springs.
// ---------------------------------------------------------------------------
const crashSim = new Simulation(config);
crashSim.reset(0, 0, 0);
for (let i = 0; i < 240; i++) crashSim.stepExplicit(neutral, 1);

crashSim.vehicle.rigidBody.position.y = 1.15;
crashSim.vehicle.rigidBody.orientation = PhysicsMath.quatFromEuler(
  18 * Math.PI / 180,
  0,
  72 * Math.PI / 180
);
crashSim.vehicle.rigidBody.velocity = PhysicsMath.vec3(14, -7.5, 18);
crashSim.vehicle.rigidBody.angularVelocity = PhysicsMath.vec3(3.5, 2.2, 5.4);

let maxPostStepPenetrationM = 0;
let maxCrashAngularSpeed = 0;
let crashNonFinite = 0;
let maxCrashHeaveM = 0;
for (let i = 0; i < 600; i++) {
  const state = crashSim.stepExplicit(neutral, 1);
  const probe = probeChassisContact(crashSim.vehicle);
  maxPostStepPenetrationM = Math.max(maxPostStepPenetrationM, probe.maxPenetrationM);
  maxCrashAngularSpeed = Math.max(
    maxCrashAngularSpeed,
    PhysicsMath.vec3Length(crashSim.vehicle.rigidBody.angularVelocity)
  );
  maxCrashHeaveM = Math.max(maxCrashHeaveM, Math.abs(state.heave));
  if (!finiteVehicle(crashSim)) crashNonFinite++;

  for (const wheel of state.wheels) {
    assert(wheel.verticalTravelM <= 0.140001, `wipeout exceeded bump travel: ${wheel.verticalTravelM}`);
    assert(wheel.verticalTravelM >= -0.120001, `wipeout exceeded droop travel: ${wheel.verticalTravelM}`);
  }
}

assert(crashNonFinite === 0, `wipeout produced ${crashNonFinite} non-finite samples`);
assert(
  maxPostStepPenetrationM < 0.055,
  `body shell penetrated road after crash projection: ${maxPostStepPenetrationM} m`
);
assert(maxCrashAngularSpeed <= 12.01, `wipeout angular speed exceeded crash ceiling: ${maxCrashAngularSpeed}`);
assert(maxCrashHeaveM < 0.85, `wipeout let chassis separate excessively from road/wheels: ${maxCrashHeaveM} m`);

console.log(JSON.stringify({
  uprightSpin: {
    maxAngularSpeedRadS: maxSpinAngularSpeed,
    maxHeaveM: maxSpinHeaveM,
    nonFiniteSamples: spinNonFinite,
  },
  wipeout: {
    maxPostStepPenetrationM,
    maxAngularSpeedRadS: maxCrashAngularSpeed,
    maxHeaveM: maxCrashHeaveM,
    nonFiniteSamples: crashNonFinite,
  },
  status: 'passed',
}, null, 2));
