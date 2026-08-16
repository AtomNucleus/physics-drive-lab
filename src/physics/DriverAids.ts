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

  constructor(config: DriverAidsConfig) {
    this.config = { ...config };
  }

  public reset() {
    this.absPressureStates = [1, 1, 1, 1];
    this.absHoldTimers = [0, 0, 0, 0];
    this.absActive = false;
    this.tcsActive = false;
    this.tcsThrottleReduction = 0;
    this.currentCenterSteerAngle = 0;
  }

  public updateSteering(
    steerInput: number,
    forwardSpeedMs: number,
    dt: number
  ): { steerFL: number; steerFR: number; centerAngle: number } {
    const speedRatio = Math.min(1.0, Math.abs(forwardSpeedMs) / 38.0);
    const maxAllowedAngle = this.config.maxSteerAngle * (1.0 - speedRatio * this.config.steerSpeedReduction);
    const targetCenterAngle = -steerInput * maxAllowedAngle;

    const steerStep = this.config.steerSpeed * dt;
    if (Math.abs(targetCenterAngle - this.currentCenterSteerAngle) <= steerStep) {
      this.currentCenterSteerAngle = targetCenterAngle;
    } else {
      this.currentCenterSteerAngle += Math.sign(targetCenterAngle - this.currentCenterSteerAngle) * steerStep;
    }

    const delta = this.currentCenterSteerAngle;
    if (Math.abs(delta) < 1e-4) return { steerFL: 0, steerFR: 0, centerAngle: 0 };

    const L = this.config.wheelbase;
    const W = this.config.trackWidth;
    const tanDelta = Math.tan(Math.abs(delta));

    let deltaInner = Math.atan((L * tanDelta) / Math.max(0.1, L - 0.5 * W * tanDelta));
    let deltaOuter = Math.atan((L * tanDelta) / Math.max(0.1, L + 0.5 * W * tanDelta));

    deltaInner = PhysicsMath.lerp(Math.abs(delta), deltaInner, this.config.ackermannRatio);
    deltaOuter = PhysicsMath.lerp(Math.abs(delta), deltaOuter, this.config.ackermannRatio);

    const steerFL = delta > 0 ? deltaOuter : -deltaInner;
    const steerFR = delta > 0 ? deltaInner : -deltaOuter;
    return { steerFL, steerFR, centerAngle: this.currentCenterSteerAngle };
  }

  /**
   * Four-channel slip-regulating ABS.
   *
   * This controller regulates pressure continuously around the peak of the tire's
   * longitudinal slip curve instead of using a deep-lock bang/bang threshold.
   * That prevents the unrealistic case where more pedal produces LESS braking.
   */
  public updateABS(
    wheelSlipRatios: [number, number, number, number],
    wheelAngularVelocities: [number, number, number, number],
    speedMs: number,
    isBraking: boolean,
    dt: number
  ): [number, number, number, number] {
    if (this.config.absMode === 'OFF' || !isBraking || speedMs < 1.4) {
      this.absActive = false;
      this.absPressureStates = [1, 1, 1, 1];
      this.absHoldTimers = [0, 0, 0, 0];
      return this.absPressureStates;
    }

    const isSport = this.config.absMode === 'SPORT';
    const targetSlip = isSport ? 0.145 : 0.12;
    const deadband = isSport ? 0.018 : 0.014;
    const minPressure = isSport ? 0.24 : 0.20;
    let anyIntervention = false;

    for (let i = 0; i < 4; i++) {
      const slipMag = Math.max(0, -wheelSlipRatios[i]);
      const nearLock = speedMs > 3.0 && Math.abs(wheelAngularVelocities[i]) < 0.35;
      const effectiveSlip = nearLock ? Math.max(slipMag, 0.9) : slipMag;
      let p = this.absPressureStates[i];

      if (effectiveSlip > targetSlip + deadband) {
        // Overslip: proportional pressure release. Deep lock releases faster, but
        // no single 120 Hz tick can throw away most of the available brake force.
        const over = effectiveSlip - (targetSlip + deadband);
        const releaseRate = 3.0 + Math.min(6.0, over * 10.0); // pressure units / s
        p = Math.max(minPressure, p - releaseRate * dt);
        anyIntervention = true;
      } else if (effectiveSlip < targetSlip - deadband) {
        // Reapply briskly enough to keep the tire near peak µ rather than spending
        // long periods at low hydraulic pressure after each ABS event.
        const under = (targetSlip - deadband) - effectiveSlip;
        const reapplyRate = 2.4 + Math.min(2.6, under * 12.0);
        p = Math.min(1.0, p + reapplyRate * dt);
        anyIntervention = anyIntervention || p < 0.995;
      } else {
        // In the target band, hold pressure. This is the useful braking state.
        anyIntervention = anyIntervention || p < 0.995;
      }

      this.absPressureStates[i] = p;
      this.absHoldTimers[i] = 0;
    }

    this.absActive = anyIntervention;
    return this.absPressureStates;
  }

  public updateTCS(
    drivenWheelSlipRatios: number[],
    dt: number
  ): { throttleMultiplier: number; tcsActive: boolean } {
    if (this.config.tcsMode === 'OFF') {
      this.tcsActive = false;
      this.tcsThrottleReduction = 0;
      return { throttleMultiplier: 1.0, tcsActive: false };
    }

    const isSport = this.config.tcsMode === 'SPORT';
    const tcsThreshold = isSport ? 0.19 : 0.12;
    const maxSlip = Math.max(0, ...drivenWheelSlipRatios);

    if (maxSlip > tcsThreshold) {
      const excess = maxSlip - tcsThreshold;
      const gain = isSport ? 2.0 : 3.0;
      const maxReduction = isSport ? 0.72 : 0.88;
      const targetReduction = Math.min(maxReduction, excess * gain);
      const response = isSport ? 10.0 : 16.0;
      this.tcsThrottleReduction += (targetReduction - this.tcsThrottleReduction) * Math.min(1.0, response * dt);
      this.tcsActive = true;
    } else {
      const recovery = isSport ? 6.5 : 5.0;
      this.tcsThrottleReduction = Math.max(0, this.tcsThrottleReduction - recovery * dt);
      this.tcsActive = this.tcsThrottleReduction > 0.04;
    }

    const minimumThrottle = isSport ? 0.26 : 0.12;
    const throttleMultiplier = Math.max(minimumThrottle, 1.0 - this.tcsThrottleReduction);
    return { throttleMultiplier, tcsActive: this.tcsActive };
  }
}
