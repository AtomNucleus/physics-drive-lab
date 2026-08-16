import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../src/physics/m5G90';

const MPH = 2.2369362921;
const G = 9.81;
const cfg:any = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES };
const targetMph = 46.9;
const targetMs = targetMph / MPH;
const zero:any = { throttle:0, brake:0, steer:0, handbrake:false, shiftUp:false, shiftDown:false };

function run(steer:number) {
  const sim = new Simulation(cfg);
  sim.reset(0,0,0);
  sim.vehicle.rigidBody.velocity = { x:0, y:0, z:targetMs };
  sim.vehicle.wheels.forEach((w:any)=>w.reset(targetMs));
  sim.vehicle.powertrain.isAutomatic = false;
  sim.vehicle.powertrain.gear = 4;
  const wheelOmega = targetMs / cfg.wheelRadius;
  const engineOmega = wheelOmega * cfg.forwardGearRatios[3] * cfg.finalDriveRatio;
  sim.vehicle.powertrain.engineRpm = engineOmega * 30 / Math.PI;

  const rows:any[] = [];
  for(let i=0;i<120*5;i++) {
    const local = sim.vehicle.rigidBody.getLocalVelocity();
    const speed = Math.abs(local.z);
    const error = targetMs - speed;
    const throttle = Math.max(0, Math.min(0.50, 0.10 + error * 0.12));
    const brake = error < -0.7 ? Math.max(0, Math.min(0.18, -error * 0.035)) : 0;
    const st = sim.stepExplicit({...zero, steer, throttle, brake},1);
    if(i >= 120*3.5) {
      rows.push({
        speedMph: st.speedMs * MPH,
        latG: Math.abs(sim.vehicle.rigidBody.acceleration.x / G),
        yawRate: Math.abs(st.yawRate),
        frontSlipDeg: ((Math.abs(st.wheels[0].slipAngle)+Math.abs(st.wheels[1].slipAngle))*0.5)*180/Math.PI,
        rearSlipDeg: ((Math.abs(st.wheels[2].slipAngle)+Math.abs(st.wheels[3].slipAngle))*0.5)*180/Math.PI,
        throttle,
        brake,
      });
    }
  }
  const avg=(k:string)=>rows.reduce((a,r)=>a+r[k],0)/rows.length;
  return {
    steer,
    speedMph:avg('speedMph'),
    latG:avg('latG'),
    yawRateDegS:avg('yawRate')*180/Math.PI,
    frontSlipDeg:avg('frontSlipDeg'),
    rearSlipDeg:avg('rearSlipDeg'),
    meanThrottle:avg('throttle'),
    meanBrake:avg('brake'),
  };
}

const sweeps=[0.10,0.12,0.14,0.16,0.18,0.20,0.22,0.24,0.26].map(run);
console.log(JSON.stringify({targetMph,sweeps,peakG:Math.max(...sweeps.map(x=>x.latG))},null,2));
