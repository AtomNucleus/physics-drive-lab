import assert from 'node:assert/strict';
import { mapMobileSteeringDirection } from '../../components/mobileControls';

assert.equal(
  mapMobileSteeringDirection('left'),
  'steerRight',
  'The left touch arrow must map to the simulator steering action that turns the car left.'
);

assert.equal(
  mapMobileSteeringDirection('right'),
  'steerLeft',
  'The right touch arrow must map to the simulator steering action that turns the car right.'
);

assert.notEqual(
  mapMobileSteeringDirection('left'),
  mapMobileSteeringDirection('right'),
  'Left and right touch arrows must never resolve to the same steering action.'
);

console.log('MobileControlsTests: PASS');
