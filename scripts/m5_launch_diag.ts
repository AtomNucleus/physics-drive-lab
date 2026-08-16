import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../src/physics/m5G90';

const DT=1/120, MPH=2.2369362921, G=9.81;
const base:any={...DEFAULT_VEHICLE_CONFIG,...BMW_M5_2025_OVERRIDES};
const zero:any={throttle:0,brake:0,steer:0,handbrake:false,shiftUp:false,shiftDown:false};

function run(name:string,tcs:'OFF'|'SPORT'|'FULL',fill:number,preload:boolean,frontRatio:number,longSigma=0.08,longForceSigma=0.045,drivelineInertia=0.55){
  const cfg:any={...base,lowSpeedTorqueFillNm:fill,centerFrontTorqueRatio:frontRatio,longitudinalRelaxationLength:longSigma,longitudinalForceRelaxationLength:longForceSigma,drivelineInputInertia:drivelineInertia};
  const sim=new Simulation(cfg); sim.reset(0,0,0); sim.vehicle.powertrain.isAutomatic=true;
  sim.vehicle.driverAids.config.tcsMode=tcs as any;
  if(preload){
    for(let i=0;i<180;i++) sim.stepExplicit({...zero,throttle:1,brake:1},1);
  }
  const staged={rpm:sim.vehicle.getState().rpm,boost:sim.vehicle.getState().turboBoostPsi};
  let t30:number|null=null,t60:number|null=null,t100:number|null=null;
  let peakLongG=0,meanFirstSecG=0,nFirst=0,maxSlip=0,tcsTicks=0,maxDriveTorque=0;
  for(let i=0;i<1440;i++){
    const st=sim.stepExplicit({...zero,throttle:1},1); const t=(i+1)*DT;
    const mph=st.speedMs*MPH;
    const longG=Math.abs(sim.vehicle.rigidBody.acceleration.z/G);
    peakLongG=Math.max(peakLongG,longG);
    if(t<=1){meanFirstSecG+=longG;nFirst++;}
    maxSlip=Math.max(maxSlip,...st.wheels.map((w:any)=>Math.max(0,w.slipRatio)));
    if(sim.vehicle.driverAids.tcsActive)tcsTicks++;
    maxDriveTorque=Math.max(maxDriveTorque,Math.abs(sim.vehicle.powertrain.deliveredDriveshaftTorque));
    if(t30===null&&mph>=30)t30=t;
    if(t60===null&&mph>=60)t60=t;
    if(t100===null&&mph>=100)t100=t;
    if(st.rpm>6800&&sim.vehicle.powertrain.gear<8)sim.vehicle.powertrain.shiftUp();
    if(t100!==null)break;
  }
  return {name,tcs,fill,preload,frontRatio,longSigma,longForceSigma,drivelineInertia,staged,t30,t60,t100,peakLongG,meanFirstSecG:meanFirstSecG/Math.max(1,nFirst),maxSlip,tcsActivePct:tcsTicks/Math.max(1,(t100??12)*120)*100,maxDriveTorque};
}

const out=[
  run('baseline-long-080','SPORT',300,true,.32,.08,.045,.55),
  run('long-050','SPORT',300,true,.32,.05,.028,.55),
  run('long-065','SPORT',300,true,.32,.065,.036,.55),
  run('long-100','SPORT',300,true,.32,.10,.055,.55),
  run('long-120','SPORT',300,true,.32,.12,.066,.55),
  run('front-40-long065','SPORT',300,true,.40,.065,.036,.55),
  run('inertia-040-long065','SPORT',300,true,.36,.065,.036,.40),
  run('inertia-070-long065','SPORT',300,true,.36,.065,.036,.70),
  run('tcs-off-long065','OFF',300,true,.36,.065,.036,.55),
];
console.log(JSON.stringify(out,null,2));
