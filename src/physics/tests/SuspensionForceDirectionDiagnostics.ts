import { Simulation } from '../Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import type { VehicleConfig, ControlInputs } from '../../types';

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

for (let i = 0; i < 480; i++) sim.stepExplicit(neutral, 1);
const state = sim.vehicle.getState();
const totalChassisSupport = sim.vehicle.suspension.states.reduce((sum, s) => sum + s.chassisForceN, 0);
const pitchPropulsionEstimate = totalChassisSupport * Math.sin(state.pitch);
const wheelLongForces = state.wheels.map((w) => ({ id: w.id, fx: w.forceVectorLong, omega: w.angularVelocity }));

console.log(JSON.stringify({
  pitchRad: state.pitch,
  pitchDeg: state.pitch * 180 / Math.PI,
  vx: state.vx,
  vz: state.vz,
  speedMs: Math.hypot(state.vx, state.vz),
  totalChassisSupport,
  pitchPropulsionEstimate,
  deliveredDriveshaftTorque: state.engineTorqueDelivered,
  wheelLongForces,
}, null, 2));
