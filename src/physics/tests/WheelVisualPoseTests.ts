import { computeWheelVisualPose } from '../../graphics/wheelVisualPose';

const assert = (condition: boolean, message: string) => {
  if (!condition) throw new Error(message);
};
const assertNear = (actual: number, expected: number, tolerance: number, message: string) => {
  assert(Math.abs(actual - expected) <= tolerance, `${message}: ${actual} vs ${expected}`);
};

const base = {
  chassisHeaveM: 0,
  chassisPitchRad: 0,
  chassisRollRad: 0,
  mountX: -0.84,
  mountZ: 1.5,
  suspensionTravelM: 0.03,
  tireSquishM: 0.018,
  sidewallDeflectionM: 0.006,
  isLeft: true,
  camberRad: -1.5 * Math.PI / 180,
  visualWheelRadiusM: 0.33,
};

const flat = computeWheelVisualPose(base);
assertNear(flat.x, -0.846, 1e-9, 'flat wheel X changed unexpectedly');
assertNear(flat.y, 0.342, 1e-9, 'flat wheel Y changed unexpectedly');
assertNear(flat.z, 1.5, 1e-9, 'flat wheel Z changed unexpectedly');
assertNear(flat.rotationX, 0, 1e-9, 'flat wheel inherited pitch unexpectedly');

const roll = 55 * Math.PI / 180;
const pitch = 24 * Math.PI / 180;
const heave = 0.11;
const crashPose = computeWheelVisualPose({
  ...base,
  chassisHeaveM: heave,
  chassisPitchRad: pitch,
  chassisRollRad: roll,
});

// The wheel center must follow the rotated chassis pickup plus only the bounded
// suspension/tire offset. This is the invariant that prevents the body appearing
// to tear away from the springs during a wipeout.
const expectedRelativeWheelY = 0.33 + base.suspensionTravelM - base.tireSquishM;
assertNear(
  crashPose.y - heave - crashPose.mountY,
  expectedRelativeWheelY,
  1e-9,
  'wheel/body vertical attachment invariant broke under crash roll/pitch'
);
assert(crashPose.crashAttachmentBlend > 0.5, 'hard rollover did not blend wheel orientation into chassis orientation');
assert(Math.abs(crashPose.mountY) > 0.25, 'test rollover did not generate meaningful rotated pickup height');
assert(Math.abs(crashPose.rotationZ) > Math.abs(base.camberRad), 'crash wheel did not inherit chassis roll');

console.log(JSON.stringify({
  flat,
  crashPose,
  attachmentInvariantM: crashPose.y - heave - crashPose.mountY,
  status: 'passed',
}, null, 2));
