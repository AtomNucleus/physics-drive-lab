import { AssistMode } from '../types';
import { PhysicsMath } from './math/PhysicsMath';

export interface DriverAidsConfig {
  absMode: AssistMode;
  tcsMode: AssistMode;
  wheelbase: number;
  trackWidth: number;
  ackermannRatio: number; // 0 = parallel, 1 = 100% Ackermann
  maxSteerAngle: number; // radians
  steerSpeed: number; // rad/s
  steerSpeedReduction: number;
}

export class DriverAidsSystem {
  public config: DriverAidsConfig;

  // ABS Internal States for 4 corners
  private absPressureStates: [number, number, number, number] = [1, 1, 1, 1];
  private absHoldTimers: [number, number, number, number] = [0, 0, 0, 0];
  public absActive: boolean = false;

  // TCS Internal States
  public tcsActive: boolean = false;
  private tcsThrottleReduction: number = 0;

  // Steering Dynamic State
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

  /**
   * Update steering rack with Ackermann geometry and speed-sensitive filtering
   */
  public updateSteering(
    steerInput: number,
    forwardSpeedMs: number,
    dt: number
  ): {
    steerFL: number;
    steerFR: number;
    centerAngle: number;
  } {
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
    if (Math.abs(delta) < 1e-4) {
      return { steerFL: 0, steerFR: 0, centerAngle: 0 };
    }

    const L = this.config.wheelbase;
    const W = this.config.trackWidth;
    const tanDelta = Math.tan(Math.abs(delta));

    let deltaInner = Math.atan((L * tanDelta) / Math.max(0.1, L - 0.5 * W * tanDelta));
    let deltaOuter = Math.atan((L * tanDelta) / Math.max(0.1, L + 0.5 * W * tanDelta));

    const ackermann = this.config.ackermannRatio;
    deltaInner = PhysicsMath.lerp(Math.abs(delta), deltaInner, ackermann);
    deltaOuter = PhysicsMath.lerp(Math.abs(delta), deltaOuter, ackermann);

    let steerFL = 0;
    let steerFR = 0;

    if (delta > 0) {
      steerFL = deltaOuter;
      steerFR = deltaInner;
    } else {
      steerFL = -deltaInner;
      steerFR = -deltaOuter;
    }

    return {
      steerFL,
      steerFR,
      centerAngle: this.currentCenterSteerAngle,
    };
  }

  /**
   * Four-channel slip-regulating ABS.
   *
   * The tire model peaks in longitudinal force around 13-15% slip. ABS therefore
   * regulates near that region instead of waiting for the wheel to reach deep
   * lockup. SPORT permits a little more slip/rotation than FULL, but both stay
   * near the useful part of the tire curve.
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

    let anyIntervention = false;

    // Hysteresis band around the tire's peak-slip region.
    // FULL: tighter road-car regulation. SPORT: slightly more slip and pedal feel.
    const dumpThreshold = this.config.absMode === 'SPORT' ? -0.17 : -0.145;
    const holdThreshold = this.config.absMode === 'SPORT' ? -0.135 : -0.115;
    const releaseRate = this.config.absMode === 'SPORT' ? 20.0 : 24.0;
    const reapplyRate = this.config.absMode === 'SPORT' ? 10.0 : 8.0;
    const minimumPressure = this.config.absMode === 'SPORT' ? 0.16 : 0.12;

    for (let i = 0; i < 4; i++) {
      const slip = wheelSlipRatios[i];
      const omega = wheelAngularVelocities[i];
      const nearLock = speedMs > 3.0 && Math.abs(omega) < 0.35;

      if (slip < dumpThreshold || nearLock) {
        // RELEASE: rapidly reduce pressure when the tire passes peak braking slip.
        this.absPressureStates[i] = Math.max(minimumPressure, this.absPressureStates[i] - releaseRate * dt);
        this.absHoldTimers[i] = 0.012; // ~12 ms hydraulic/valve hold
        anyIntervention = true;
      } else if (slip < holdThreshold || this.absHoldTimers[i] > 0) {
        // HOLD: sit near peak slip rather than instantly reapplying into another lock.
        this.absHoldTimers[i] = Math.max(0, this.absHoldTimers[i] - dt);
        anyIntervention = anyIntervention || this.absPressureStates[i] < 0.995;
      } else {
        // REAPPLY progressively when the wheel has recovered.
        this.absPressureStates[i] = Math.min(1.0, this.absPressureStates[i] + reapplyRate * dt);
        anyIntervention = anyIntervention || this.absPressureStates[i] < 0.995;
      }
    }

    this.absActive = anyIntervention;
    return this.absPressureStates;
  }

  /** Update Traction Control System */
  public updateTCS(
    drivenWheelSlipRatios: number[],
    dt: number
  ): {
    throttleMultiplier: number;
    tcsActive: boolean;
  } {
    if (this.config.tcsMode === 'OFF') {
      this.tcsActive = false;
      this.tcsThrottleReduction = 0;
      return { throttleMultiplier: 1.0, tcsActive: false };
    }

    // Peak longitudinal traction is well below the old 28% SPORT threshold.
    // SPORT still permits useful wheelspin, while FULL stays close to peak grip.
    const tcsThreshold = this.config.tcsMode === 'SPORT' ? 0.18 : 0.11;
    const maxSlip = Math.max(0, ...drivenWheelSlipRatios);

    if (maxSlip > tcsThreshold) {
      const excess = maxSlip - tcsThreshold;
      const targetReduction = Math.min(0.88, excess * 3.4);
      this.tcsThrottleReduction += (targetReduction - this.tcsThrottleReduction) * Math.min(1.0, 18.0 * dt);
      this.tcsActive = true;
    } else {
      this.tcsThrottleReduction = Math.max(0, this.tcsThrottleReduction - 5.0 * dt);
      this.tcsActive = this.tcsThrottleReduction > 0.04;
    }

    const throttleMultiplier = Math.max(0.12, 1.0 - this.tcsThrottleReduction);
    return { throttleMultiplier, tcsActive: this.tcsActive };
  }
}
