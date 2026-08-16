import { WheelDynamics } from '../WheelDynamics';
import { Simulation } from '../Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import type { VehicleConfig, ControlInputs } from '../../types';

const DT = 1 / 120;

function assert(condition: boolean, message: string): asserts condition {
  if (!condition) throw new Error(message);
}

function assertFinite(value: number, label: string) {
  assert(Number.isFinite(value), `${label} must remain finite, got ${value}`);
}

function makeM5Wheel(id: 'FL' | 'FR', isLeft: boolean) {
  return new WheelDynamics({
    id,
    isFront: true,
    isLeft,
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

function testStationaryCamberDoesNotCreatePlanarForce() {
  for (const [id, isLeft] of [['FL', true], ['FR', false]] as const) {
    const wheel = makeM5Wheel(id, isLeft);
    wheel.reset(0);
    let maxPlanarForce = 0;
    let maxSkidIntensity = 0;

    for (let i = 0; i < 720; i++) {
      const out = wheel.update(
        0,
        0,
        6200,
        -1.5,
        0,
        0,
        0,
        1.0,
        0.015,
        DT
      );
      maxPlanarForce = Math.max(maxPlanarForce, Math.hypot(out.fx, out.fy));
      maxSkidIntensity = Math.max(maxSkidIntensity, out.skidIntensity);
      assert(!out.isSkidding, `${id}: stationary tire must never be marked skidding`);
    }

    assert(maxPlanarForce < 1.0, `${id}: stationary cambered tire invented ${maxPlanarForce.toFixed(2)} N planar force`);
    assert(maxSkidIntensity === 0, `${id}: stationary tire produced skid intensity ${maxSkidIntensity}`);
  }
}

function testCreepBrushForceIsBoundedAndDissipative() {
  const wheel = makeM5Wheel('FL', true);
  wheel.reset(0);

  let maxForce = 0;
  let sawSkid = false;
  for (let i = 0; i < 240; i++) {
    // A 3 cm/s sideways disturbance is representative of numerical/body-settle
    // motion at parking speed. It should be resisted smoothly, not classified as
    // a high-energy slide.
    const vy = i < 120 ? 0.03 : -0.03;
    const out = wheel.update(0, vy, 6200, -1.5, 0, 0, 0, 1, 0.015, DT);
    maxForce = Math.max(maxForce, Math.hypot(out.fx, out.fy));
    sawSkid ||= out.isSkidding;
    assertFinite(out.fx, 'creep fx');
    assertFinite(out.fy, 'creep fy');
    assert(out.frictionLimit > 5000, `creep friction limit unexpectedly low: ${out.frictionLimit}`);
    assert(Math.hypot(out.fx, out.fy) <= out.frictionLimit * 1.001, 'creep force exceeded friction circle');
  }

  assert(!sawSkid, '3 cm/s parking-speed patch motion must not trigger tire smoke/skid state');
  assert(maxForce < 7000, `creep brush force became unbounded: ${maxForce.toFixed(1)} N`);
}

function testRealWheelspinStillProducesSkid() {
  const wheel = makeM5Wheel('FL', true);
  wheel.reset(0);

  let sawSkid = false;
  let peakIntensity = 0;
  for (let i = 0; i < 240; i++) {
    const out = wheel.update(0, 0, 6200, -1.5, 3500, 0, 0, 1, 0.015, DT);
    sawSkid ||= out.isSkidding;
    peakIntensity = Math.max(peakIntensity, out.skidIntensity);
  }

  assert(sawSkid, 'real high-energy wheelspin must still enter the skid state');
  assert(peakIntensity > 0.05, `real wheelspin skid intensity too low: ${peakIntensity}`);
}

function runStationaryVehicleScenario(steer: number, label: string) {
  const config = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES } as VehicleConfig;
  const sim = new Simulation(config);
  sim.reset(0, 0, 0);

  const neutral: ControlInputs = {
    throttle: 0,
    brake: 0,
    steer: 0,
    handbrake: false,
    shiftUp: false,
    shiftDown: false,
  };

  // Let unsprung/sprung states settle before applying steering so this test measures
  // horizontal tire stability rather than the initial vertical suspension drop.
  for (let i = 0; i < 480; i++) sim.stepExplicit(neutral, 1);

  const start = sim.vehicle.getState();
  const startTemp = start.wheels.map((w) => w.temperature);
  const steering: ControlInputs = { ...neutral, steer };

  let maxPlanarSpeed = 0;
  let maxYawRate = 0;
  let maxSkidIntensity = 0;
  let skidFrames = 0;

  for (let i = 0; i < 720; i++) {
    const state = sim.stepExplicit(steering, 1);
    const planarSpeed = Math.hypot(state.vx, state.vz);
    maxPlanarSpeed = Math.max(maxPlanarSpeed, planarSpeed);
    maxYawRate = Math.max(maxYawRate, Math.abs(state.yawRate));

    for (const wheel of state.wheels) {
      assertFinite(wheel.forceVectorLat, `${label} ${wheel.id} lateral force`);
      assertFinite(wheel.forceVectorLong, `${label} ${wheel.id} longitudinal force`);
      maxSkidIntensity = Math.max(maxSkidIntensity, wheel.skidIntensity);
      if (wheel.isSkidding) skidFrames++;
    }
  }

  const end = sim.vehicle.getState();
  const horizontalDisplacement = Math.hypot(end.x - start.x, end.z - start.z);
  const maxTempRise = Math.max(...end.wheels.map((w, i) => w.temperature - startTemp[i]));

  assert(skidFrames === 0, `${label}: stationary full-lock steering reported ${skidFrames} skid/smoke wheel-frames`);
  assert(maxSkidIntensity === 0, `${label}: stationary full-lock steering reached skid intensity ${maxSkidIntensity}`);
  assert(maxPlanarSpeed < 0.12, `${label}: chassis shook/moved at ${maxPlanarSpeed.toFixed(3)} m/s while parked`);
  assert(maxYawRate < 0.12, `${label}: chassis yaw oscillation reached ${maxYawRate.toFixed(3)} rad/s while parked`);
  assert(horizontalDisplacement < 0.08, `${label}: parked car migrated ${horizontalDisplacement.toFixed(3)} m under steering alone`);
  assert(maxTempRise < 0.10, `${label}: stationary steering heated a tire by ${maxTempRise.toFixed(3)} C`);

  return { maxPlanarSpeed, maxYawRate, horizontalDisplacement, maxTempRise };
}

function testStationaryFullLockVehicle() {
  const left = runStationaryVehicleScenario(1, 'full-left');
  const right = runStationaryVehicleScenario(-1, 'full-right');

  // Both steering directions should be comparably stable; this also catches a
  // left/right force transform regression.
  const speedAsymmetry = Math.abs(left.maxPlanarSpeed - right.maxPlanarSpeed);
  const displacementAsymmetry = Math.abs(left.horizontalDisplacement - right.horizontalDisplacement);
  assert(speedAsymmetry < 0.05, `left/right stationary speed response differs by ${speedAsymmetry.toFixed(3)} m/s`);
  assert(displacementAsymmetry < 0.05, `left/right stationary migration differs by ${displacementAsymmetry.toFixed(3)} m`);
}

function main() {
  const tests: Array<[string, () => void]> = [
    ['stationary camber creates no planar force', testStationaryCamberDoesNotCreatePlanarForce],
    ['parking-speed brush force is bounded and non-smoking', testCreepBrushForceIsBoundedAndDissipative],
    ['real wheelspin still produces skid', testRealWheelspinStillProducesSkid],
    ['full-lock parked M5 remains stable left and right', testStationaryFullLockVehicle],
  ];

  for (const [name, test] of tests) {
    test();
    console.log(`PASS ${name}`);
  }

  console.log(`PASS all ${tests.length} stationary/low-speed tire regression tests`);
}

main();
