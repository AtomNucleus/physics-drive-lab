import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../src/physics/m5G90';

const DT=1/120, MPH=2.2369362921, G=9.81;
const base:any={...DEFAULT_VEHICLE_CONFIG,...BMW_M5_2025_OVERRIDES};
const zero:any={throttle:0,brake:0,steer:0,handbrake:false,shiftUp:false,shiftDown:false};

function launch(longSigma:number,inertia:number,frontRatio:number,threshold:number,response:number,gain:number){
  const cfg:any={
    ...base,
    longitudinalRelaxationLength:longSigma,
    longitudinalForceRelaxationLength:longSigma*0.55,
    drivelineInputInertia:inertia,
    centerFrontTorqueRatio:frontRatio,
    tcsSportSlipThreshold:threshold,
    tcsSportResponse:response,
    tcsSportGain:gain,
  };
  const sim=new Simulation(cfg); sim.reset(0,0,0); sim.vehicle.powertrain.isAutomatic=true;
  for(let i=0;i<180;i++)sim.stepExplicit({...zero,throttle:1,brake:1},1);
  let t30:number|null=null,t60:number|null=null,t100:number|null=null,maxSlip=0,sumG=0,nG=0,tcsTicks=0;
  for(let i=0;i<1440;i++){
    const st=sim.stepExplicit({...zero,throttle:1},1); const t=(i+1)*DT; const mph=st.speedMs*MPH;
    if(t<=1){sumG+=Math.abs(sim.vehicle.rigidBody.acceleration.z/G);nG++;}
    maxSlip=Math.max(maxSlip,...st.wheels.map((w:any)=>Math.max(0,w.slipRatio)));
    if(sim.vehicle.driverAids.tcsActive)tcsTicks++;
    if(t30===null&&mph>=30)t30=t;
    if(t60===null&&mph>=60)t60=t;
    if(t100===null&&mph>=100)t100=t;
    if(st.rpm>6800&&sim.vehicle.powertrain.gear<8)sim.vehicle.powertrain.shiftUp();
    if(t100!==null)break;
  }
  const a=t30??20,b=t60??20,c=t100??20;
  // C/D published launch numbers omit 0.2 s of rollout. These are the equivalent
  // true-standing-start targets for this stopwatch.
  const score=Math.abs(a-1.3)*2.4+Math.abs(b-3.2)*2.0+Math.abs(c-6.9)+Math.max(0,maxSlip-.30)*.12;
  return {longSigma,inertia,frontRatio,threshold,response,gain,t30,t60,t100,meanFirstSecG:sumG/Math.max(1,nG),maxSlip,tcsActivePct:tcsTicks/Math.max(1,(c*120))*100,score};
}

function skidpad(longSigma:number){
  let peak=0;
  for(const steer of [0.12,0.14,0.16,0.18,0.20,0.22]){
    const cfg:any={...base,longitudinalRelaxationLength:longSigma,longitudinalForceRelaxationLength:longSigma*0.55};
    const sim=new Simulation(cfg);sim.reset(0,0,0);
    const v=48/MPH;sim.vehicle.rigidBody.velocity={x:0,y:0,z:v};sim.vehicle.wheels.forEach((w:any)=>w.reset(v));sim.vehicle.powertrain.gear=0;
    for(let i=0;i<60;i++)sim.stepExplicit(zero,1);
    let sum=0,n=0;
    for(let i=0;i<360;i++){
      sim.stepExplicit({...zero,steer},1);
      if(i>240){sum+=Math.abs(sim.vehicle.rigidBody.acceleration.x/G);n++;}
    }
    peak=Math.max(peak,sum/Math.max(1,n));
  }
  return {longSigma,peakG:peak};
}

const rows:any[]=[];
for(const longSigma of [0.12,0.16,0.19])
for(const inertia of [0.35,0.40,0.48,0.55])
for(const frontRatio of [0.36,0.40,0.44])
for(const threshold of [0.10,0.12,0.14,0.16])
for(const response of [18,30])
  rows.push(launch(longSigma,inertia,frontRatio,threshold,response,2.6));
rows.sort((a,b)=>a.score-b.score);
console.log(JSON.stringify({standingStartTargets:{t30:1.3,t60:3.2,t100:6.9},skidpadByLongSigma:[0.12,0.16,0.19].map(skidpad),topLaunch:rows.slice(0,20)},null,2));
