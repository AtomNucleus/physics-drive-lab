import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../src/physics/m5G90';

const DT=1/120, MPH=2.2369362921, G=9.81;
const base:any={
  ...DEFAULT_VEHICLE_CONFIG,
  ...BMW_M5_2025_OVERRIDES,
  longitudinalRelaxationLength:0.12,
  longitudinalForceRelaxationLength:0.066,
  drivelineInputInertia:0.35,
  centerFrontTorqueRatio:0.40,
};
const zero:any={throttle:0,brake:0,steer:0,handbrake:false,shiftUp:false,shiftDown:false};

function run(shiftDurationSec:number, shiftTorqueMultiplier:number){
  const sim=new Simulation(base); sim.reset(0,0,0); sim.vehicle.powertrain.isAutomatic=true;
  Object.assign(sim.vehicle.driverAids.config as any, {
    tcsSportSlipThreshold:0.16,
    tcsSportResponse:30,
    tcsSportGain:2.6,
  });
  Object.assign(sim.vehicle.powertrain.config as any, {shiftDurationSec,shiftTorqueMultiplier});
  for(let i=0;i<180;i++) sim.stepExplicit({...zero,throttle:1,brake:1},1);
  let t30:number|null=null,t60:number|null=null,t100:number|null=null,qtr:number|null=null,trap:number|null=null;
  let maxSlip=0,sumG=0,nG=0; const z0=sim.vehicle.getState().z;
  for(let i=0;i<1800;i++){
    const st=sim.stepExplicit({...zero,throttle:1},1); const t=(i+1)*DT, mph=st.speedMs*MPH;
    if(t<=1){sumG+=Math.abs(sim.vehicle.rigidBody.acceleration.z/G);nG++;}
    maxSlip=Math.max(maxSlip,...st.wheels.map((w:any)=>Math.max(0,w.slipRatio)));
    if(t30===null&&mph>=30)t30=t;
    if(t60===null&&mph>=60)t60=t;
    if(t100===null&&mph>=100)t100=t;
    const dist=Math.abs(st.z-z0);
    if(qtr===null&&dist>=402.336){qtr=t;trap=mph;}
    if(st.rpm>6800&&sim.vehicle.powertrain.gear<8)sim.vehicle.powertrain.shiftUp();
    if(t100!==null&&qtr!==null)break;
  }
  const a=t30??20,b=t60??20,c=t100??20,q=qtr??20;
  const score=Math.abs(a-1.3)*2.4+Math.abs(b-3.2)*2+Math.abs(c-6.9)+Math.abs(q-11.1)*.8+Math.abs((trap??0)-130)*.03;
  return {shiftDurationSec,shiftTorqueMultiplier,t30,t60,t100,quarterMile:qtr,trapMph:trap,meanFirstSecG:sumG/Math.max(1,nG),maxSlip,score};
}
const rows:any[]=[];
for(const duration of [0.05,0.07,0.09,0.11,0.13,0.16])
for(const torqueMultiplier of [0.15,0.30,0.45,0.60,0.80]) rows.push(run(duration,torqueMultiplier));
rows.sort((a,b)=>a.score-b.score);
console.log(JSON.stringify({standingTargets:{t30:1.3,t60:3.2,t100:6.9,quarterMile:11.1,trapMph:130},top:rows.slice(0,20)},null,2));
