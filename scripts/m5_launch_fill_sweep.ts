import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../src/physics/m5G90';

const DT=1/120, MPH=2.2369362921, G=9.81;
const base:any={...DEFAULT_VEHICLE_CONFIG,...BMW_M5_2025_OVERRIDES};
const zero:any={throttle:0,brake:0,steer:0,handbrake:false,shiftUp:false,shiftDown:false};

function run(fill:number,fade:number,launchRpm:number,inertia:number,frontRatio:number){
  const cfg:any={
    ...base,
    maxTorque:700,
    lowSpeedTorqueFillNm:fill,
    torqueFillFadeRpm:fade,
    launchControlRpm:launchRpm,
    drivelineInputInertia:inertia,
    centerFrontTorqueRatio:frontRatio,
    longitudinalRelaxationLength:.12,
    longitudinalForceRelaxationLength:.066,
  };
  const sim=new Simulation(cfg); sim.reset(0,0,0); sim.vehicle.powertrain.isAutomatic=true;
  Object.assign(sim.vehicle.driverAids.config as any,{tcsSportSlipThreshold:.16,tcsSportResponse:30,tcsSportGain:2.6});
  Object.assign(sim.vehicle.powertrain.config as any,{shiftDurationSec:.07,shiftTorqueMultiplier:.80});
  for(let i=0;i<180;i++)sim.stepExplicit({...zero,throttle:1,brake:1},1);
  const stagedRpm=sim.vehicle.powertrain.engineRpm, stagedBoost=sim.vehicle.powertrain.turboBoostPsi;
  let t30:number|null=null,t60:number|null=null,t100:number|null=null,q:number|null=null,trap:number|null=null,maxSlip=0,sumG=0,nG=0;
  const z0=sim.vehicle.getState().z;
  for(let i=0;i<1800;i++){
    const st=sim.stepExplicit({...zero,throttle:1},1), t=(i+1)*DT, mph=st.speedMs*MPH, dist=Math.abs(st.z-z0);
    if(t<=1){sumG+=Math.abs(sim.vehicle.rigidBody.acceleration.z/G);nG++;}
    maxSlip=Math.max(maxSlip,...st.wheels.map((w:any)=>Math.max(0,w.slipRatio)));
    if(t30===null&&mph>=30)t30=t;
    if(t60===null&&mph>=60)t60=t;
    if(t100===null&&mph>=100)t100=t;
    if(q===null&&dist>=402.336){q=t;trap=mph;}
    if(st.rpm>6800&&sim.vehicle.powertrain.gear<8)sim.vehicle.powertrain.shiftUp();
    if(t100!==null&&q!==null)break;
  }
  const a=t30??20,b=t60??20,c=t100??20,d=q??20,v=trap??0;
  const score=Math.abs(a-1.3)*3+Math.abs(b-3.2)*2+Math.abs(c-6.9)*1.5+Math.abs(d-11.1)+Math.abs(v-130)*.16+Math.max(0,maxSlip-1.1)*.08;
  return {fill,fade,launchRpm,inertia,frontRatio,stagedRpm,stagedBoost,t30,t60,t100,quarterMile:q,trapMph:trap,meanFirstSecG:sumG/Math.max(1,nG),maxSlip,score};
}

const rows:any[]=[];
for(const fill of [300,350,400,450,500,550,600])
for(const fade of [2600,2800,3000,3200,3400,3800])
for(const launchRpm of [2800,3000,3200,3400])
for(const inertia of [.35,.40])
for(const frontRatio of [.40,.44]) rows.push(run(fill,fade,launchRpm,inertia,frontRatio));
rows.sort((a,b)=>a.score-b.score);
console.log(JSON.stringify({standingTargets:{t30:1.3,t60:3.2,t100:6.9,quarterMile:11.1,trapMph:130},top:rows.slice(0,30)},null,2));
