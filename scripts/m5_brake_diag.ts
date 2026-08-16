import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../src/physics/m5G90';

const MPH=2.2369362921, G=9.81;
const base:any={...DEFAULT_VEHICLE_CONFIG,...BMW_M5_2025_OVERRIDES};
const zero:any={throttle:0,brake:0,steer:0,handbrake:false,shiftUp:false,shiftDown:false};

function brake(startMph:number,brakeForce:number,aeroScale:number,mode:'SPORT'|'FULL'='SPORT'){
  const cfg:any={...base,brakeForce,aeroDownforceFront:base.aeroDownforceFront*aeroScale,aeroDownforceRear:base.aeroDownforceRear*aeroScale};
  const sim=new Simulation(cfg); sim.reset(0,0,0);
  const v=startMph/MPH;
  sim.vehicle.rigidBody.velocity={x:0,y:0,z:v};
  sim.vehicle.wheels.forEach((w:any)=>w.reset(v));
  sim.vehicle.powertrain.gear=0;
  for(let i=0;i<60;i++)sim.stepExplicit(zero,1);
  sim.vehicle.driverAids.config.absMode=mode as any;
  const z0=sim.vehicle.getState().z;
  let samples=0,sumG=0,sumP=0,sumSlip=0,maxSlip=0;
  for(let i=0;i<1440;i++){
    const st=sim.stepExplicit({...zero,brake:1},1);
    if(st.speedMs>2){
      const g=Math.abs(sim.vehicle.rigidBody.acceleration.z/G);
      sumG+=g;samples++;
      const ps=sim.vehicle.brakes.pressureModulators; sumP+=ps.reduce((a,b)=>a+b,0)/4;
      const slips=st.wheels.map((w:any)=>Math.abs(w.slipRatio)); sumSlip+=slips.reduce((a:number,b:number)=>a+b,0)/4; maxSlip=Math.max(maxSlip,...slips);
    }
    if(st.speedMs<.8)break;
  }
  const d=Math.abs(sim.vehicle.getState().z-z0);
  return {startMph,brakeForce,aeroScale,mode,distanceFt:d/.3048,meanG:sumG/Math.max(1,samples),meanPressure:sumP/Math.max(1,samples),meanSlip:sumSlip/Math.max(1,samples),maxSlip};
}

const out:any[]=[];
for(const brakeForce of [10000,11000,12000,13000,14000,15000]){
  for(const aeroScale of [0,0.25,0.5,1]){
    const b70=brake(70,brakeForce,aeroScale);
    const b100=brake(100,brakeForce,aeroScale);
    const score=Math.abs(b70.distanceFt-157)+Math.abs(b100.distanceFt-324);
    out.push({brakeForce,aeroScale,b70,b100,score});
  }
}
out.sort((a,b)=>a.score-b.score);
console.log(JSON.stringify(out.slice(0,12),null,2));
