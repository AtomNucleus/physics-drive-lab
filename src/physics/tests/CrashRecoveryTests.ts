import * as THREE from 'three';
import { CarRenderer } from '../../graphics/carRenderer';
import { WheelDynamics } from '../WheelDynamics';
import { projectTireShearOntoSurface } from '../Vehicle';
import { PhysicsMath } from '../math/PhysicsMath';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';

const DT = 1 / 120;

function assert(condition: boolean, message: string): asserts condition {
  if (!condition) throw new Error(message);
}

function near(a: number, b: number, eps: number = 1e-7) {
  return Math.abs(a - b) <= eps;
}

function makeWheel() {
  return new WheelDynamics({
    id: 'FL',
    isFront: true,
    isLeft: true,
    radius: 0.369,
    inertia: 2.10,
    tireConfig: {
      baseGrip: 1.21,
      stiffnessB: 15.0,
      loadSensitivity: 0.000030,
      slideFrictionMultiplier: 0.83,
      relaxationLength: 0.19,
      longitudinalRelaxationLength: 0.12,
      longitudinalForceRelaxationLength: 0.066,
      pneumaticTrailMax: 0.030,
      camberStiffness: 85,
      optimalTemp: 75,
      basePressurePsi: 35,
      sidewallStiffness: 230000,
      verticalStiffness: 280000,
      referenceLoadN: 6200,
    },
  });
}

function testTireShearStaysInRoadPlane() {
  const normal = PhysicsMath.vec3Normalize({ x: 0.18, y: 0.96, z: -0.21 });
  const raw = { x: 4200, y: 3100, z: -5600 };
  const tangent = projectTireShearOntoSurface(raw, normal);
  const normalLeak = Math.abs(PhysicsMath.vec3Dot(tangent, normal));
  assert(normalLeak < 1e-8, `tire shear leaked ${normalLeak} N into road normal`);
  assert(
    PhysicsMath.vec3Length(tangent) <= PhysicsMath.vec3Length(raw) + 1e-8,
    'road-plane projection created force'
  );
}

function testPostSpinContactPatchSettles() {
  const wheel = makeWheel();
  wheel.reset(12);

  let sawRealSkid = false;
  for (let i = 0; i < 180; i++) {
    const out = wheel.update(12, 8, 6200, -1.5, 0, 0, 0, 1, 0.015, DT);
    sawRealSkid ||= out.isSkidding;
  }
  assert(sawRealSkid, 'high-energy spin phase did not register a skid');

  for (let i = 0; i < 300; i++) {
    const t = 1 - (i + 1) / 300;
    wheel.update(12 * t, 8 * t, 6200, -1.5, 0, 0, 0, 1, 0.015, DT);
  }

  let latePeakForce = 0;
  let lateSkidFrames = 0;
  for (let i = 0; i < 480; i++) {
    const out = wheel.update(0, 0, 6200, -1.5, 0, 0, 0, 1, 0.015, DT);
    if (i >= 240) {
      latePeakForce = Math.max(latePeakForce, Math.hypot(out.fx, out.fy));
      if (out.isSkidding || out.skidIntensity > 0) lateSkidFrames++;
    }
  }

  assert(lateSkidFrames === 0, `settled tire re-entered skid for ${lateSkidFrames} frames`);
  assert(latePeakForce < 80, `settled tire retained ${latePeakForce.toFixed(1)} N oscillatory force`);
}

function testChassisVisualPivotsAtPhysicalCg() {
  const config = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES } as any;
  const renderer = new CarRenderer('#2563eb');
  const cgOffset = config.centerOfGravityHeight + 0.35;
  const state: any = {
    x: 12,
    z: -7,
    elevationHeight: 1.4,
    heave: 0.16,
    yaw: 2.2,
    pitch: 0.92,
    roll: 1.18,
    airbrakeActive: false,
    drsActive: false,
    exhaustFlameIntensity: 0,
    wheels: [],
    showForceVectors3D: false,
  };

  renderer.update(state, config);
  renderer.rootGroup.updateMatrixWorld(true);

  assert(
    near(renderer.chassisPivotGroup.position.y, state.heave + cgOffset),
    'body pivot is not located at rigid-body CG height'
  );
  assert(
    near(renderer.chassisGroup.position.y, -cgOffset),
    'body visual is not offset from the CG pivot correctly'
  );

  const before = new THREE.Vector3();
  renderer.chassisPivotGroup.getWorldPosition(before);
  state.pitch = -1.05;
  state.roll = -1.22;
  renderer.update(state, config);
  renderer.rootGroup.updateMatrixWorld(true);
  const after = new THREE.Vector3();
  renderer.chassisPivotGroup.getWorldPosition(after);
  assert(
    before.distanceTo(after) < 1e-7,
    `CG pivot moved ${before.distanceTo(after)} m when only attitude changed`
  );
}

const tests: Array<[string, () => void]> = [
  ['tire shear remains tangent to road', testTireShearStaysInRoadPlane],
  ['post-spin contact patch settles', testPostSpinContactPatchSettles],
  ['chassis visual rotates around physical CG', testChassisVisualPivotsAtPhysicalCg],
];

for (const [name, test] of tests) {
  test();
  console.log(`PASS ${name}`);
}
console.log(`PASS all ${tests.length} crash-recovery regression tests`);
