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

const dt = 1 / 120;
const speedMs = 20; // 72 km/h: quick enough for a real turn-in transient without tire saturation.
const steerInput = 0.14;
const sim = new Simulation(config);
sim.reset(0, 0, 0);

for (let i = 0; i < 300; i++) sim.stepExplicit(neutral, 1);
sim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, speedMs);
sim.vehicle.wheels.forEach((wheel) => wheel.reset(speedMs));
for (let i = 0; i < 90; i++) sim.stepExplicit(neutral, 1);

type Sample = {
  t: number;
  frontFy: number;
  outsideTravelDelta: number;
  roll: number;
  yawRate: number;
};

const sampleState = (t: number): Sample => {
  const state = sim.vehicle.getState();
  const frontFy = Math.abs(state.wheels[0].forceVectorLat) + Math.abs(state.wheels[1].forceVectorLat);
  const outsideTravelDelta =
    0.5 * (state.wheels[1].verticalTravelM + state.wheels[3].verticalTravelM) -
    0.5 * (state.wheels[0].verticalTravelM + state.wheels[2].verticalTravelM);
  return {
    t,
    frontFy,
    outsideTravelDelta,
    roll: Math.abs(state.roll),
    yawRate: Math.abs(state.yawRate),
  };
};

const turnIn: Sample[] = [];
for (let step = 0; step < 180; step++) {
  sim.stepExplicit({ ...neutral, steer: steerInput }, 1);
  turnIn.push(sampleState((step + 1) * dt));
}

const tail = turnIn.slice(-30);
const mean = (values: number[]) => values.reduce((sum, value) => sum + value, 0) / Math.max(1, values.length);
const steady = {
  frontFy: mean(tail.map((s) => s.frontFy)),
  outsideTravelDelta: mean(tail.map((s) => s.outsideTravelDelta)),
  roll: mean(tail.map((s) => s.roll)),
  yawRate: mean(tail.map((s) => s.yawRate)),
};

const firstTimeAtFraction = (
  samples: Sample[],
  key: keyof Pick<Sample, 'frontFy' | 'outsideTravelDelta' | 'roll'>,
  target: number,
  fraction: number
): number => {
  const threshold = Math.abs(target) * fraction;
  const hit = samples.find((s) => Math.abs(s[key]) >= threshold);
  return hit?.t ?? Number.POSITIVE_INFINITY;
};

const fractionAt = (
  samples: Sample[],
  key: keyof Pick<Sample, 'frontFy' | 'outsideTravelDelta' | 'roll'>,
  target: number,
  timeSec: number
): number => {
  const index = Math.max(0, Math.min(samples.length - 1, Math.round(timeSec / dt) - 1));
  return Math.abs(samples[index][key]) / Math.max(1e-9, Math.abs(target));
};

const turnInTiming = {
  tire25: firstTimeAtFraction(turnIn, 'frontFy', steady.frontFy, 0.25),
  tire50: firstTimeAtFraction(turnIn, 'frontFy', steady.frontFy, 0.50),
  travel25: firstTimeAtFraction(turnIn, 'outsideTravelDelta', steady.outsideTravelDelta, 0.25),
  travel50: firstTimeAtFraction(turnIn, 'outsideTravelDelta', steady.outsideTravelDelta, 0.50),
  roll25: firstTimeAtFraction(turnIn, 'roll', steady.roll, 0.25),
  roll50: firstTimeAtFraction(turnIn, 'roll', steady.roll, 0.50),
};

const turnInFractions = {
  at50ms: {
    tire: fractionAt(turnIn, 'frontFy', steady.frontFy, 0.05),
    travel: fractionAt(turnIn, 'outsideTravelDelta', steady.outsideTravelDelta, 0.05),
    roll: fractionAt(turnIn, 'roll', steady.roll, 0.05),
  },
  at100ms: {
    tire: fractionAt(turnIn, 'frontFy', steady.frontFy, 0.10),
    travel: fractionAt(turnIn, 'outsideTravelDelta', steady.outsideTravelDelta, 0.10),
    roll: fractionAt(turnIn, 'roll', steady.roll, 0.10),
  },
  at250ms: {
    tire: fractionAt(turnIn, 'frontFy', steady.frontFy, 0.25),
    travel: fractionAt(turnIn, 'outsideTravelDelta', steady.outsideTravelDelta, 0.25),
    roll: fractionAt(turnIn, 'roll', steady.roll, 0.25),
  },
};

const release: Sample[] = [];
for (let step = 0; step < 180; step++) {
  sim.stepExplicit(neutral, 1);
  release.push(sampleState((step + 1) * dt));
}

const releaseFractionAt = (
  key: keyof Pick<Sample, 'frontFy' | 'outsideTravelDelta' | 'roll'>,
  target: number,
  timeSec: number
) => fractionAt(release, key, target, timeSec);

const releaseFractions = {
  at50ms: {
    tire: releaseFractionAt('frontFy', steady.frontFy, 0.05),
    travel: releaseFractionAt('outsideTravelDelta', steady.outsideTravelDelta, 0.05),
    roll: releaseFractionAt('roll', steady.roll, 0.05),
  },
  at100ms: {
    tire: releaseFractionAt('frontFy', steady.frontFy, 0.10),
    travel: releaseFractionAt('outsideTravelDelta', steady.outsideTravelDelta, 0.10),
    roll: releaseFractionAt('roll', steady.roll, 0.10),
  },
  at250ms: {
    tire: releaseFractionAt('frontFy', steady.frontFy, 0.25),
    travel: releaseFractionAt('outsideTravelDelta', steady.outsideTravelDelta, 0.25),
    roll: releaseFractionAt('roll', steady.roll, 0.25),
  },
};

const maxReleaseRoll = Math.max(...release.map((s) => s.roll));
const finalReleaseRoll = mean(release.slice(-30).map((s) => s.roll));

assert(steady.frontFy > 1500, `turn-in did not generate meaningful front lateral force: ${steady.frontFy.toFixed(0)} N`);
assert(Math.abs(steady.outsideTravelDelta) > 0.001, 'turn-in did not generate measurable outside suspension compression');
assert(steady.roll > 0.002, `turn-in did not generate measurable body roll: ${(steady.roll * 180 / Math.PI).toFixed(3)} deg`);
assert(steady.yawRate > 0.03, 'turn-in did not generate meaningful yaw rate');

// M5 turn-in target: the rack may move immediately, but the tire contact patch must
// take a set before the 2.4-ton chassis acquires its cornering attitude. These bounds
// deliberately measure normalized response rather than reducing the final roll angle.
assert(
  turnInTiming.tire50 >= 0.016 && turnInTiming.tire50 <= 0.050,
  `front tires should build to 50% over a short carcass transient, got ${(turnInTiming.tire50 * 1000).toFixed(1)} ms`
);
assert(
  turnInTiming.tire50 < turnInTiming.roll50,
  `front tire force must lead chassis roll: tire50=${turnInTiming.tire50.toFixed(3)}s roll50=${turnInTiming.roll50.toFixed(3)}s`
);
assert(
  turnInFractions.at50ms.roll < 0.12,
  `body flops over too early: ${(turnInFractions.at50ms.roll * 100).toFixed(1)}% of steady roll at 50 ms`
);
assert(
  turnInFractions.at100ms.travel > 0.12 && turnInFractions.at100ms.travel < 0.45,
  `outside suspension should be actively loading, not settled, at 100 ms: ${(turnInFractions.at100ms.travel * 100).toFixed(1)}%`
);
assert(
  turnInFractions.at100ms.roll > 0.20 && turnInFractions.at100ms.roll < 0.45,
  `body should still be loading at 100 ms: ${(turnInFractions.at100ms.roll * 100).toFixed(1)}% of steady roll`
);
assert(
  turnInFractions.at250ms.travel > 0.75 && turnInFractions.at250ms.travel < 1.25,
  `outside suspension did not settle into a cornering attitude by 250 ms: ${(turnInFractions.at250ms.travel * 100).toFixed(1)}%`
);
assert(
  turnInFractions.at250ms.roll > 0.80 && turnInFractions.at250ms.roll < 1.20,
  `body did not settle into a controlled cornering attitude by 250 ms: ${(turnInFractions.at250ms.roll * 100).toFixed(1)}%`
);

// Release target is the reverse sequence: tire lateral force sheds first while the
// already-loaded suspension and body unwind, then both settle without a snap-back.
assert(
  releaseFractions.at50ms.tire < 0.25 && releaseFractions.at50ms.travel > 0.70 && releaseFractions.at50ms.roll > 0.65,
  `steering release sequence is wrong at 50 ms: tire=${releaseFractions.at50ms.tire.toFixed(2)} travel=${releaseFractions.at50ms.travel.toFixed(2)} roll=${releaseFractions.at50ms.roll.toFixed(2)}`
);
assert(
  releaseFractions.at250ms.tire < 0.15 && releaseFractions.at250ms.travel < 0.20 && releaseFractions.at250ms.roll < 0.10,
  `car did not unwind cleanly by 250 ms: tire=${releaseFractions.at250ms.tire.toFixed(2)} travel=${releaseFractions.at250ms.travel.toFixed(2)} roll=${releaseFractions.at250ms.roll.toFixed(2)}`
);
assert(finalReleaseRoll < steady.roll * 0.05, 'body retained roll after steering release');
assert(maxReleaseRoll < steady.roll * 1.20, 'steering release produced an excessive roll spike');

console.log(JSON.stringify({
  speedKmh: speedMs * 3.6,
  steerInput,
  steady: {
    frontFyN: steady.frontFy,
    outsideTravelDeltaM: steady.outsideTravelDelta,
    rollDeg: steady.roll * 180 / Math.PI,
    yawRateDegS: steady.yawRate * 180 / Math.PI,
  },
  turnInTimingSec: turnInTiming,
  turnInFractions,
  releaseFractions,
  release: {
    maxRollDeg: maxReleaseRoll * 180 / Math.PI,
    finalRollDeg: finalReleaseRoll * 180 / Math.PI,
  },
  status: 'passed',
}, null, 2));
