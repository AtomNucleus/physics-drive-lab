import { Vehicle } from '../Vehicle';
import { PhysicsMath } from '../math/PhysicsMath';
import { ProvingGroundSurfaceProvider, type ISurfaceProvider } from '../SurfaceProvider';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';

const config: any = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES };
const dt = 1 / 120;
const idleInputs: any = { throttle: 0, brake: 0, steer: 0, handbrake: false, shiftUp: false, shiftDown: false };

class FlatSurface implements ISurfaceProvider {
  sampleSurface(_x: number, _z: number): any {
    return {
      elevation: 0,
      normal: PhysicsMath.vec3(0, 1, 0),
      slopePitch: 0,
      slopeRoll: 0,
      type: 'asphalt',
      friction: 1,
      rollingResistance: 0,
      wetness: 0,
      isKerbRumble: false,
    };
  }
}

function settle(vehicle: Vehicle, seconds = 4) {
  for (let i = 0; i < seconds / dt; i++) vehicle.step(idleInputs, dt);
}

function run(name: string, surface: ISurfaceProvider, startZ: number, speedMs: number, seconds: number) {
  const v = new Vehicle(config, surface);
  v.reset(0, startZ, 0);
  settle(v, 4);
  v.rigidBody.velocity = PhysicsMath.vec3(0, 0, speedMs);

  let maxBodyY = -Infinity;
  let minBodyY = Infinity;
  let maxHeave = -Infinity;
  let minHeave = Infinity;
  let maxAirborneWheels = 0;
  let maxSuspensionForce = 0;
  let maxHardStopForce = 0;
  let maxDamperForce = 0;
  let maxAbsVerticalVelocity = 0;
  let maxVisualBodyWheelGap = -Infinity;
  let sampleAtWorst: any = null;

  for (let i = 0; i < seconds / dt; i++) {
    v.step(idleInputs, dt);
    const state: any = v.getState();
    maxBodyY = Math.max(maxBodyY, state.y);
    minBodyY = Math.min(minBodyY, state.y);
    maxHeave = Math.max(maxHeave, state.heave);
    minHeave = Math.min(minHeave, state.heave);
    maxAirborneWheels = Math.max(maxAirborneWheels, state.airborneWheelsCount);
    maxAbsVerticalVelocity = Math.max(maxAbsVerticalVelocity, Math.abs(v.rigidBody.velocity.y));

    for (const s of v.suspension.states) {
      maxSuspensionForce = Math.max(maxSuspensionForce, s.forceNorm);
      maxHardStopForce = Math.max(maxHardStopForce, s.hardStopForceN);
      maxDamperForce = Math.max(maxDamperForce, Math.abs(s.damperForceN));
    }

    // Current renderer: root gets state.y, then chassis gets state.heave again.
    const chassisOriginWorldY = state.y + state.heave;
    const wheelCentersWorldY = state.wheels.map((w: any) => state.y + 0.33 + w.verticalTravelM - w.tireSquishM);
    const avgWheelCenterWorldY = wheelCentersWorldY.reduce((a: number, b: number) => a + b, 0) / 4;
    const visualGap = chassisOriginWorldY - avgWheelCenterWorldY;
    if (visualGap > maxVisualBodyWheelGap) {
      maxVisualBodyWheelGap = visualGap;
      sampleAtWorst = {
        t: Number(((i + 1) * dt).toFixed(3)),
        z: Number(state.z.toFixed(3)),
        speedKmh: Number(state.speedKmh.toFixed(1)),
        bodyY: Number(state.y.toFixed(4)),
        heave: Number(state.heave.toFixed(4)),
        verticalVelocity: Number(v.rigidBody.velocity.y.toFixed(4)),
        airborne: state.airborneWheelsCount,
        travel: state.wheels.map((w: any) => Number(w.verticalTravelM.toFixed(4))),
        forceN: v.suspension.states.map(s => Math.round(s.forceNorm)),
        hardStopN: v.suspension.states.map(s => Math.round(s.hardStopForceN)),
        damperN: v.suspension.states.map(s => Math.round(s.damperForceN)),
        visualGap: Number(visualGap.toFixed(4)),
      };
    }
  }

  return {
    name,
    maxBodyY: Number(maxBodyY.toFixed(4)),
    minBodyY: Number(minBodyY.toFixed(4)),
    maxHeave: Number(maxHeave.toFixed(4)),
    minHeave: Number(minHeave.toFixed(4)),
    maxAirborneWheels,
    maxSuspensionForce: Math.round(maxSuspensionForce),
    maxHardStopForce: Math.round(maxHardStopForce),
    maxDamperForce: Math.round(maxDamperForce),
    maxAbsVerticalVelocity: Number(maxAbsVerticalVelocity.toFixed(4)),
    maxVisualBodyWheelGap: Number(maxVisualBodyWheelGap.toFixed(4)),
    sampleAtWorst,
  };
}

const results = [
  run('flat-216kmh', new FlatSurface(), 0, 60, 4),
  run('proving-ground-crest-216kmh', new ProvingGroundSurfaceProvider(), 0, 60, 5),
];

for (const result of results) console.log(JSON.stringify(result, null, 2));
