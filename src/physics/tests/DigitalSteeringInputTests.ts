import assert from 'node:assert/strict';
import { digitalSteeringLimitForSpeed, updateDigitalSteeringInput } from '../DigitalSteeringInput';

const DT = 1 / 120;

assert.equal(digitalSteeringLimitForSpeed(0), 1, 'parking speed must retain full steering lock');
assert.equal(digitalSteeringLimitForSpeed(8 / 3.6), 1, '8 km/h must still allow full steering lock');
assert(Math.abs(digitalSteeringLimitForSpeed(30 / 3.6) - 0.48) < 1e-12, '30 km/h digital steering cap must be 0.48');
assert(digitalSteeringLimitForSpeed(100 / 3.6) <= 0.18 + 1e-12, '100 km/h digital steering must be strongly limited');

let left = 0;
for (let i = 0; i < 120; i++) left = updateDigitalSteeringInput(left, 1, 30 / 3.6, DT);
assert(Math.abs(left - 0.48) < 1e-12, `held left at 30 km/h must settle at +0.48, got ${left}`);

let right = 0;
for (let i = 0; i < 120; i++) right = updateDigitalSteeringInput(right, -1, 30 / 3.6, DT);
assert(Math.abs(right + 0.48) < 1e-12, `held right at 30 km/h must settle at -0.48, got ${right}`);
assert(Math.abs(left + right) < 1e-12, 'digital steering left/right limits must mirror exactly');

let parking = 0;
for (let i = 0; i < 120; i++) parking = updateDigitalSteeringInput(parking, 1, 6 / 3.6, DT);
assert(Math.abs(parking - 1) < 1e-12, '6 km/h parking/crawl steering must still reach full lock');

let release = left;
for (let i = 0; i < 60; i++) release = updateDigitalSteeringInput(release, 0, 30 / 3.6, DT);
assert(Math.abs(release) < 1e-12, 'released digital steering must return to center');

console.log('DigitalSteeringInputTests: PASS');
