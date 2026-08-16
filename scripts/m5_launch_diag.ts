import { Simulation } from '../src/physics/Simulation';
import { DEFAULT_VEHICLE_CONFIG } from '../src/physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../src/physics/m5G90';

const DT=1/120, MPH=2.2369362921, G=9.81;
const base:any={...DEFAULT_VEHICLE_CONFIG,...BMW_M5_2025_OVERRIDES};
const zero:any={throttle:0,brake:0,steer:0,handbrake:false,shiftUp:false,shiftDown:false};

function run(name:string,tcs:'OFF'|'SPORT'|'FULL',fill:number,preload:boolean,frontRatio:number){
  const cfg:any={...base,lowSpeedTorqueFillNm:fill,centerFrontTorqueRatio:frontRatio};
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
  return {name,tcs,fill,preload,frontRatio,staged,t30,t60,t100,peakLongG,meanFirstSecG:meanFirstSecG/Math.max(1,nFirst),maxSlip,tcsActivePct:tcsTicks/Math.max(1,(t100??12)*120)*100,maxDriveTorque};
}

const out=[
  run('baseline-sport','SPORT',300,true,.32),
  run('tcs-off','OFF',300,true,.32),
  run('full-tcs','FULL',300,true,.32),
  run('more-fill','SPORT',500,true,.32),
  run('less-fill','SPORT',150,true,.32),
  run('awd-40-front','SPORT',300,true,.40),
  run('no-preload','SPORT',300,false,.32),
];
console.log(JSON.stringify(out,null,2));
