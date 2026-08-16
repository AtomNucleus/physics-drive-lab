import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../src/physics/m5G90';

const DT=1/120, MPH=2.2369362921, FT=3.28084, G=9.81;
const zero:any={throttle:0,brake:0,steer:0,handbrake:false,shiftUp:false,shiftDown:false};
const base:any={...DEFAULT_VEHICLE_CONFIG,...BMW_M5_2025_OVERRIDES};

function stop(startMph:number, brakeForce:number, bias:number){
  const cfg:any={...base,brakeForce,brakeBiasFront:bias};
  const sim=new Simulation(cfg); sim.reset(0,0,0);
  const v=startMph/MPH;
  sim.vehicle.rigidBody.velocity={x:0,y:0,z:v};
  sim.vehicle.wheels.forEach((w:any)=>w.reset(v));
  sim.vehicle.powertrain.gear=0;
  sim.vehicle.driverAids.config.absMode='SPORT';
  for(let i=0;i<12;i++)sim.stepExplicit(zero,1);
  const z0=sim.vehicle.getState().z;
  let sumG=0,n=0,peakG=0,sumSlip=0,sumPressure=0;
  for(let i=0;i<2400;i++){
    const st=sim.stepExplicit({...zero,brake:1},1);
    const g=Math.abs(sim.vehicle.rigidBody.acceleration.z/G);
    peakG=Math.max(peakG,g); sumG+=g; n++;
    sumSlip += st.wheels.reduce((a:number,w:any)=>a+Math.max(0,-w.slipRatio),0)/4;
    sumPressure += (sim.vehicle.brakes.pressureModulators as number[]).reduce((a,b)=>a+b,0)/4;
    if(st.speedMs<0.15){
      return {distanceFt:Math.abs(st.z-z0)*FT,meanG:sumG/n,peakG,meanSlip:sumSlip/n,meanPressure:sumPressure/n};
    }
  }
  return {distanceFt:null,meanG:sumG/Math.max(1,n),peakG,meanSlip:sumSlip/Math.max(1,n),meanPressure:sumPressure/Math.max(1,n)};
}

const rows:any[]=[];
for(let force=10000;force<=11200;force+=100){
  for(const bias of [.58,.60,.62,.64,.66,.68]){
    const s70=stop(70,force,bias), s100=stop(100,force,bias);
    const e70=s70.distanceFt===null?999:Math.abs(s70.distanceFt-157);
    const e100=s100.distanceFt===null?999:Math.abs(s100.distanceFt-324);
    rows.push({force,bias,s70,s100,errorFt:e70+e100,maxErrorFt:Math.max(e70,e100)});
  }
}
rows.sort((a,b)=>a.errorFt-b.errorFt || a.maxErrorFt-b.maxErrorFt);
console.log(JSON.stringify({targets:{s70:157,s100:324},top:rows.slice(0,30)},null,2));
