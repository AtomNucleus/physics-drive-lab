
import { Simulation } from '../Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { PhysicsMath } from '../math/PhysicsMath';

const assert = (condition: boolean, message: string) => {
  if (!condition) throw new Error(message);
};

const config = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES } as any;
const neutral = { throttle: 0, brake: 0, steer: 0, handbrake: false, shiftUp: false, shiftDown: false };

function runTurn(steer: number) {
  const sim = new Simulation(config);
  sim.reset(0, 0, 0);
  for (let i = 0; i < 240; i++) sim.stepExplicit(neutral, 1);
  const speedMs = 18;
  sim.vehicle.rigidBody.velocity = PhysicsMath.vec3(0, 0, speedMs);
  sim.vehicle.wheels.forEach((wheel) => wheel.reset(speedMs));
  for (let i = 0; i < 60; i++) sim.stepExplicit(neutral, 1);

  let state = sim.vehicle.getState();
  for (let i = 0; i < 120; i++) state = sim.stepExplicit({ ...neutral, steer }, 1);
  const leftLoadN = state.wheels[0].suspensionForce + state.wheels[2].suspensionForce;
  const rightLoadN = state.wheels[1].suspensionForce + state.wheels[3].suspensionForce;
  return { state, leftLoadN, rightLoadN };
}

const left = runTurn(0.18);
assert(left.state.x > 0.5, `LEFT command must move toward vehicle-left (+X), got x=${left.state.x}`);
assert(left.state.yaw > 0.03, `LEFT command must produce positive yaw, got yaw=${left.state.yaw}`);
assert(left.state.actualSteerAngle > 0, `LEFT command must produce positive rack angle, got ${left.state.actualSteerAngle}`);
assert(left.rightLoadN > left.leftLoadN, `LEFT turn must load RIGHT/outside tires: left=${left.leftLoadN} right=${left.rightLoadN}`);

const right = runTurn(-0.18);
assert(right.state.x < -0.5, `RIGHT command must move toward vehicle-right (-X), got x=${right.state.x}`);
assert(right.state.yaw < -0.03, `RIGHT command must produce negative yaw, got yaw=${right.state.yaw}`);
assert(right.state.actualSteerAngle < 0, `RIGHT command must produce negative rack angle, got ${right.state.actualSteerAngle}`);
assert(right.leftLoadN > right.rightLoadN, `RIGHT turn must load LEFT/outside tires: left=${right.leftLoadN} right=${right.rightLoadN}`);

console.log(JSON.stringify({
  left: { x: left.state.x, yawDeg: left.state.yaw * 180 / Math.PI, leftLoadN: left.leftLoadN, rightLoadN: left.rightLoadN },
  right: { x: right.state.x, yawDeg: right.state.yaw * 180 / Math.PI, leftLoadN: right.leftLoadN, rightLoadN: right.rightLoadN },
  status: 'passed',
}, null, 2));
