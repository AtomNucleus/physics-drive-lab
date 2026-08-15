import { AssistMode } from '../types';
import { PhysicsMath } from './math/PhysicsMath';

export interface DriverAidsConfig {
  absMode: AssistMode;
  tcsMode: AssistMode;
  wheelbase: number;
  trackWidth: number;
  ackermannRatio: number;
  maxSteerAngle: number;
  steerSpeed: number;
  steerSpeedReduction: number;
}

export class DriverAidsSystem {
  public config: DriverAidsConfig;
  private absPressureStates: [number, number, number, number] = [1, 1, 1, 1];
  private absHoldTimers: [number, number, number, number] = [0, 0, 0, 0];
  public absActive: boolean = false;
  public tcsActive: boolean = false;
  private tcsThrottleReduction: number = 0;
  public currentCenterSteerAngle: number = 0;

  constructor(config: DriverAidsConfig) { this.config = { ...config }; }
  public reset() { this.absPressureStates=[1,1,1,1]; this.absHoldTimers=[0,0,0,0]; this.absActive=false; this.tcsActive=false; this.tcsThrottleReduction=0; this.currentCenterSteerAngle=0; }

  public updateSteering(steerInput:number, forwardSpeedMs:number, dt:number): {steerFL:number; steerFR:number; centerAngle:number} {
    const speedRatio=Math.min(1.0,forwardSpeedMs/38.0);
    const maxAllowedAngle=this.config.maxSteerAngle*(1.0-speedRatio*this.config.steerSpeedReduction);
    const targetCenterAngle=-steerInput*maxAllowedAngle;
    const steerStep=this.config.steerSpeed*dt;
    if(Math.abs(targetCenterAngle-this.currentCenterSteerAngle)<=steerStep) this.currentCenterSteerAngle=targetCenterAngle;
    else this.currentCenterSteerAngle+=Math.sign(targetCenterAngle-this.currentCenterSteerAngle)*steerStep;
    const delta=this.currentCenterSteerAngle;
    if(Math.abs(delta)<1e-4) return {steerFL:0,steerFR:0,centerAngle:0};
    const L=this.config.wheelbase, W=this.config.trackWidth, tanDelta=Math.tan(Math.abs(delta));
    let deltaInner=Math.atan((L*tanDelta)/Math.max(0.1,L-0.5*W*tanDelta));
    let deltaOuter=Math.atan((L*tanDelta)/Math.max(0.1,L+0.5*W*tanDelta));
    const ackermann=this.config.ackermannRatio;
    deltaInner=PhysicsMath.lerp(Math.abs(delta),deltaInner,ackermann);
    deltaOuter=PhysicsMath.lerp(Math.abs(delta),deltaOuter,ackermann);
    let steerFL=0, steerFR=0;
    if(delta>0){steerFL=deltaOuter;steerFR=deltaInner;} else {steerFL=-deltaInner;steerFR=-deltaOuter;}
    return {steerFL,steerFR,centerAngle:this.currentCenterSteerAngle};
  }

  public updateABS(wheelSlipRatios:[number,number,number,number], wheelAngularVelocities:[number,number,number,number], speedMs:number, isBraking:boolean, dt:number): [number,number,number,number] {
    if(this.config.absMode==='OFF'||!isBraking||speedMs<1.4){this.absActive=false;this.absPressureStates=[1,1,1,1];return this.absPressureStates;}
    let anyIntervention=false; const lockupThreshold=this.config.absMode==='SPORT'?-0.26:-0.15;
    for(let i=0;i<4;i++){const slip=wheelSlipRatios[i],omega=wheelAngularVelocities[i]; if(slip<lockupThreshold||(speedMs>3.0&&omega<0.2)){this.absPressureStates[i]=Math.max(0.12,this.absPressureStates[i]-18.0*dt);this.absHoldTimers[i]=0.04;anyIntervention=true;} else if(this.absHoldTimers[i]>0){this.absHoldTimers[i]-=dt;} else {this.absPressureStates[i]=Math.min(1.0,this.absPressureStates[i]+12.0*dt);}}
    this.absActive=anyIntervention; return this.absPressureStates;
  }

  public updateTCS(drivenWheelSlipRatios:number[],dt:number): {throttleMultiplier:number;tcsActive:boolean} {
    if(this.config.tcsMode==='OFF'){this.tcsActive=false;this.tcsThrottleReduction=0;return {throttleMultiplier:1.0,tcsActive:false};}
    const tcsThreshold=this.config.tcsMode==='SPORT'?0.28:0.14; const maxSlip=Math.max(0,...drivenWheelSlipRatios);
    if(maxSlip>tcsThreshold){const excess=maxSlip-tcsThreshold;const targetReduction=Math.min(0.85,excess*2.8);this.tcsThrottleReduction+=(targetReduction-this.tcsThrottleReduction)*Math.min(1.0,16.0*dt);this.tcsActive=true;} else {this.tcsThrottleReduction=Math.max(0,this.tcsThrottleReduction-6.0*dt);this.tcsActive=this.tcsThrottleReduction>0.05;}
    return {throttleMultiplier:Math.max(0.15,1.0-this.tcsThrottleReduction),tcsActive:this.tcsActive};
  }
}
