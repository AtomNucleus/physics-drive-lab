import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../src/physics/m5G90';

const DT=1/120, MPH=2.2369362921, G=9.81;
const base:any={...DEFAULT_VEHICLE_CONFIG,...BMW_M5_2025_OVERRIDES};
const zero:any={throttle:0,brake:0,steer:0,handbrake:false,shiftUp:false,shiftDown:false};

function run(maxTorque:number,fill:number,fade:number,inertia:number,frontRatio:number,longSigma:number){
  const cfg:any={
    ...base,
    maxTorque,
    lowSpeedTorqueFillNm:fill,
    torqueFillFadeRpm:fade,
    drivelineInputInertia:inertia,
    centerFrontTorqueRatio:frontRatio,
    longitudinalRelaxationLength:longSigma,
    longitudinalForceRelaxationLength:longSigma*0.55,
  };
  const sim=new Simulation(cfg); sim.reset(0,0,0); sim.vehicle.powertrain.isAutomatic=true;
  Object.assign(sim.vehicle.driverAids.config as any,{tcsSportSlipThreshold:.16,tcsSportResponse:30,tcsSportGain:2.6});
  for(let i=0;i<180;i++)sim.stepExplicit({...zero,throttle:1,brake:1},1);
  let t30:number|null=null,t60:number|null=null,t100:number|null=null,qTime:number|null=null,qTrap:number|null=null;
  const z0=sim.vehicle.getState().z;
  let maxSlip=0,sumFirstG=0,nFirst=0;
  for(let i=0;i<1800;i++){
    const st=sim.stepExplicit({...zero,throttle:1},1); const t=(i+1)*DT;
    const mph=st.speedMs*MPH; const dist=Math.abs(st.z-z0);
    const g=Math.abs(sim.vehicle.rigidBody.acceleration.z/G);
    if(t<=1){sumFirstG+=g;nFirst++;}
    maxSlip=Math.max(maxSlip,...st.wheels.map((w:any)=>Math.max(0,w.slipRatio)));
    if(t30===null&&mph>=30)t30=t;
    if(t60===null&&mph>=60)t60=t;
    if(t100===null&&mph>=100)t100=t;
    if(qTime===null&&dist>=402.336){qTime=t;qTrap=mph;}
    if(st.rpm>6800&&sim.vehicle.powertrain.gear<8)sim.vehicle.powertrain.shiftUp();
    if(t100!==null&&qTime!==null)break;
  }
  const a=t30??20,b=t60??20,c=t100??20,q=qTime??20,trap=qTrap??0;
  // C/D's published acceleration excludes roughly 0.2 s of rollout.
  const score=Math.abs(a-1.30)*1.5+Math.abs(b-3.20)*2+Math.abs(c-6.90)*1.5+Math.abs(q-11.10)+Math.abs(trap-130)*.10+Math.max(0,maxSlip-.80)*.08;
  return {maxTorque,fill,fade,inertia,frontRatio,longSigma,t30,t60,t100,quarterMile:qTime,trapMph:qTrap,meanFirstSecG:sumFirstG/Math.max(1,nFirst),maxSlip,score};
}

const rows:any[]=[];
for(const maxTorque of [700,725,750,775])
for(const fill of [250,300,350])
for(const fade of [3400,4500,5500])
for(const inertia of [.35,.40,.48])
for(const frontRatio of [.40,.44])
for(const longSigma of [.12,.16])
  rows.push(run(maxTorque,fill,fade,inertia,frontRatio,longSigma));
rows.sort((a,b)=>a.score-b.score);
console.log(JSON.stringify({standingTargets:{t30:1.3,t60:3.2,t100:6.9,quarterMile:11.1,trapMph:130},top:rows.slice(0,30)},null,2));
