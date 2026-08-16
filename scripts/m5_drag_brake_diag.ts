import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../src/physics/m5G90';

const MPH=2.2369362921, G=9.81;
const base:any={...DEFAULT_VEHICLE_CONFIG,...BMW_M5_2025_OVERRIDES};
const zero:any={throttle:0,brake:0,steer:0,handbrake:false,shiftUp:false,shiftDown:false};

function brake(startMph:number,brakeForce:number,dragCoeff:number,downforceScale:number){
  const cfg:any={
    ...base,
    brakeForce,
    aeroDragCoeff:dragCoeff,
    aeroDownforceFront:base.aeroDownforceFront*downforceScale,
    aeroDownforceRear:base.aeroDownforceRear*downforceScale,
  };
  const sim=new Simulation(cfg); sim.reset(0,0,0);
  const v=startMph/MPH;
  sim.vehicle.rigidBody.velocity={x:0,y:0,z:v};
  sim.vehicle.wheels.forEach((w:any)=>w.reset(v));
  sim.vehicle.powertrain.gear=0;
  for(let i=0;i<60;i++)sim.stepExplicit(zero,1);
  sim.vehicle.driverAids.config.absMode='SPORT' as any;
  const z0=sim.vehicle.getState().z;
  let samples=0,sumG=0,sumP=0,sumSlip=0,maxSlip=0;
  for(let i=0;i<1440;i++){
    const st=sim.stepExplicit({...zero,brake:1},1);
    if(st.speedMs>2){
      sumG+=Math.abs(sim.vehicle.rigidBody.acceleration.z/G); samples++;
      sumP+=sim.vehicle.brakes.pressureModulators.reduce((a,b)=>a+b,0)/4;
      const slips=st.wheels.map((w:any)=>Math.abs(w.slipRatio));
      sumSlip+=slips.reduce((a:number,b:number)=>a+b,0)/4; maxSlip=Math.max(maxSlip,...slips);
    }
    if(st.speedMs<.8)break;
  }
  const d=Math.abs(sim.vehicle.getState().z-z0);
  return {distanceFt:d/.3048,meanG:sumG/Math.max(1,samples),meanPressure:sumP/Math.max(1,samples),meanSlip:sumSlip/Math.max(1,samples),maxSlip};
}

const out:any[]=[];
for(const brakeForce of [10750,11000,11250,11500,11750,12000]){
  for(const dragCoeff of [0,0.10,0.20,0.30,0.35]){
    for(const downforceScale of [0,0.10,0.25,0.50,1]){
      const b70=brake(70,brakeForce,dragCoeff,downforceScale);
      const b100=brake(100,brakeForce,dragCoeff,downforceScale);
      const score=Math.abs(b70.distanceFt-157)+Math.abs(b100.distanceFt-324);
      out.push({brakeForce,dragCoeff,downforceScale,b70,b100,score});
    }
  }
}
out.sort((a,b)=>a.score-b.score);
console.log(JSON.stringify(out.slice(0,24),null,2));
