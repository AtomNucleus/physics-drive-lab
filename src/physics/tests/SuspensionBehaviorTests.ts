import { PhysicsMath } from '../math/PhysicsMath';
import {
  SuspensionSystem,
  progressiveSpringIncrement,
  damperForceForVelocity,
  bumpStopForceForDisplacement,
  type SuspensionCornerConfig,
} from '../Suspension';

const assert = (condition: boolean, message: string) => {
  if (!condition) throw new Error(message);
};

const assertNear = (actual: number, expected: number, tolerance: number, message: string) => {
  assert(Math.abs(actual - expected) <= tolerance, `${message}: ${actual} vs ${expected}`);
};

const baseRate = 62000;
const maxBump = 0.14;
const lowTravelSlope = progressiveSpringIncrement(0.03, baseRate, maxBump, 0.65) / 0.03;
const highTravelSlope = (
  progressiveSpringIncrement(0.12, baseRate, maxBump, 0.65) -
  progressiveSpringIncrement(0.09, baseRate, maxBump, 0.65)
) / 0.03;
assert(highTravelSlope > lowTravelSlope * 1.10, 'spring force curve must become stiffer deeper into bump travel');

const bumpDamper = damperForceForVelocity(0.25, 5200, 3000, 6500, 4000);
const reboundDamper = damperForceForVelocity(-0.25, 5200, 3000, 6500, 4000);
assert(bumpDamper > 0, 'compression damping must add upward support force');
assert(reboundDamper < 0, 'rebound damping must oppose extension');
assert(Math.abs(reboundDamper) > Math.abs(bumpDamper), 'M5 low/mid-speed rebound should be stronger than bump damping');

assertNear(bumpStopForceForDisplacement(0.10, 0.14, 0.80, 70000), 0, 1e-9, 'bump stop engaged before threshold');
const earlyBumpStop = bumpStopForceForDisplacement(0.12, 0.14, 0.80, 70000);
const deepBumpStop = bumpStopForceForDisplacement(0.135, 0.14, 0.80, 70000);
assert(deepBumpStop > earlyBumpStop * 2, 'bump stop must ramp progressively near full jounce');

const corner: SuspensionCornerConfig = {
  restLength: 0.34,
  springStiffness: 62000,
  dampingLowSpeed: 5200,
  dampingHighSpeed: 3000,
  dampingRebound: 6500,
  bumpStopStiffness: 70000,
  bumpStopThreshold: 0.80,
  maxDroop: 0.12,
  maxBump: 0.14,
  staticCamberDeg: -1.5,
  camberGainDegPerMeter: 7.5,
  antiDiveSquatRatio: 0.45,
};

const hardpoints = [
  PhysicsMath.vec3(-0.84, 0, 1.5),
  PhysicsMath.vec3(0.84, 0, 1.5),
  PhysicsMath.vec3(-0.84, 0, -1.5),
  PhysicsMath.vec3(0.84, 0, -1.5),
] as [ReturnType<typeof PhysicsMath.vec3>, ReturnType<typeof PhysicsMath.vec3>, ReturnType<typeof PhysicsMath.vec3>, ReturnType<typeof PhysicsMath.vec3>];

const flatSurface = () => ({
  elevation: 0,
  normal: PhysicsMath.vec3(0, 1, 0),
});

const suspension = new SuspensionSystem();
const configs: [SuspensionCornerConfig, SuspensionCornerConfig, SuspensionCornerConfig, SuspensionCornerConfig] = [
  corner, corner, corner, corner,
];
const zero = PhysicsMath.vec3();
const flatOrientation = PhysicsMath.quatFromEuler(0, 0, 0);
const rolledOrientation = PhysicsMath.quatFromEuler(0, 0, 3 * Math.PI / 180);

// Two identical steps remove the initial damper transient and give a quasi-static baseline.
suspension.update(hardpoints, PhysicsMath.vec3(0, 0.80, 0), flatOrientation, zero, zero, flatSurface, configs, 0, 0, 0, 0.369, 280000, 1 / 120);
suspension.update(hardpoints, PhysicsMath.vec3(0, 0.80, 0), flatOrientation, zero, zero, flatSurface, configs, 0, 0, 0, 0.369, 280000, 1 / 120);
const flatLoads = suspension.states.map((state) => state.forceNorm);
assertNear(flatLoads[0], flatLoads[1], 1e-6, 'flat front axle must be left/right symmetric');
assertNear(flatLoads[2], flatLoads[3], 1e-6, 'flat rear axle must be left/right symmetric');

// Roll the actual hardpoints and then let the damper transient settle. The spring loads
// must split left/right without any visual body-roll helper.
suspension.update(hardpoints, PhysicsMath.vec3(0, 0.80, 0), rolledOrientation, zero, zero, flatSurface, configs, 0, 0, 0, 0.369, 280000, 1 / 120);
suspension.update(hardpoints, PhysicsMath.vec3(0, 0.80, 0), rolledOrientation, zero, zero, flatSurface, configs, 0, 0, 0, 0.369, 280000, 1 / 120);
const rolledLoads = suspension.states.map((state) => state.forceNorm);
assert(rolledLoads[0] > rolledLoads[1], 'rolled chassis must create a front left/right normal-load split through geometry');
assert(rolledLoads[2] > rolledLoads[3], 'rolled chassis must create a rear left/right normal-load split through geometry');

// Force the chassis beyond available jounce: wheel travel itself must stop at maxBump,
// while the bump stop / hard stop takes the extra compression load.
suspension.update(hardpoints, PhysicsMath.vec3(0, 0.55, 0), flatOrientation, zero, zero, flatSurface, configs, 0, 0, 0, 0.369, 280000, 1 / 120);
assertNear(suspension.states[0].displacement, corner.maxBump, 1e-9, 'suspension exceeded configured max bump travel');
assert(suspension.states[0].atCompressionLimit, 'compression-limit state was not reported');
assert(suspension.states[0].bumpStopForceN > 0, 'bump stop did not engage near full travel');
assert(suspension.states[0].hardStopForceN > 0, 'hard stop did not resist motion past physical travel');

console.log(JSON.stringify({
  progressiveLowTravelSlope: lowTravelSlope,
  progressiveHighTravelSlope: highTravelSlope,
  bumpDamperForceN: bumpDamper,
  reboundDamperForceN: reboundDamper,
  flatLoadsN: flatLoads,
  rolledLoadsN: rolledLoads,
  status: 'passed',
}, null, 2));
