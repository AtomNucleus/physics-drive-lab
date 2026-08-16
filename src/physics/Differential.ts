import { DifferentialType, DrivetrainType } from '../types';
import { PhysicsMath } from './math/PhysicsMath';

export interface DifferentialConfig {
  type: DifferentialType;
  powerRamp: number; // 0..1 (e.g. 0.70)
  coastRamp: number; // 0..1 (e.g. 0.35)
  preloadTorque: number; // Nm (e.g. 45)
  drivetrain: DrivetrainType; // 'RWD' | 'FWD' | 'AWD'
  frontTorqueRatio?: number;
}

export class DifferentialSystem {
  public config: DifferentialConfig;

  constructor(config: DifferentialConfig) {
    this.config = { ...config };
  }

  /**
   * Split total driveshaft torque across 4 wheels based on drivetrain and differential physics
   *
   * @param inputTorque Driveshaft input torque (Nm)
   * @param wheelSpeeds Angular velocities of [FL, FR, RL, RR] (rad/s)
   */
  public distributeTorque(
    inputTorque: number,
    wheelSpeeds: [number, number, number, number]
  ): {
    wheelTorques: [number, number, number, number];
    pinionSpeed: number;
  } {
    const [omegaFL, omegaFR, omegaRL, omegaRR] = wheelSpeeds;
    const torques: [number, number, number, number] = [0, 0, 0, 0];

    if (this.config.drivetrain === 'FWD') {
      const diffOut = this.solveAxleDifferential(inputTorque, omegaFL, omegaFR);
      torques[0] = diffOut.torqueLeft;
      torques[1] = diffOut.torqueRight;
      return { wheelTorques: torques, pinionSpeed: diffOut.carrierSpeed };
    }

    if (this.config.drivetrain === 'RWD') {
      const diffOut = this.solveAxleDifferential(inputTorque, omegaRL, omegaRR);
      torques[2] = diffOut.torqueLeft;
      torques[3] = diffOut.torqueRight;
      return { wheelTorques: torques, pinionSpeed: diffOut.carrierSpeed };
    }

    // Adaptive center coupling: rear-biased baseline, then send more torque
    // forward when rear-axle overspeed indicates that the rear tires need help.
    const baseFrontRatio = PhysicsMath.clamp(this.config.frontTorqueRatio ?? 0.40, 0.20, 0.50);
    const frontOmega = (Math.abs(omegaFL) + Math.abs(omegaFR)) * 0.5;
    const rearOmega = (Math.abs(omegaRL) + Math.abs(omegaRR)) * 0.5;
    const axleSpeedRef = Math.max(1.0, (frontOmega + rearOmega) * 0.5);
    const rearOverspeed = (rearOmega - frontOmega) / axleSpeedRef;
    const frontRatio = PhysicsMath.clamp(baseFrontRatio + rearOverspeed * 0.28, 0.20, 0.50);
    const rearRatio = 1.0 - frontRatio;

    const frontDiff = this.solveAxleDifferential(inputTorque * frontRatio, omegaFL, omegaFR);
    const rearDiff = this.solveAxleDifferential(inputTorque * rearRatio, omegaRL, omegaRR);

    torques[0] = frontDiff.torqueLeft;
    torques[1] = frontDiff.torqueRight;
    torques[2] = rearDiff.torqueLeft;
    torques[3] = rearDiff.torqueRight;

    const carrierAvg = frontDiff.carrierSpeed * frontRatio + rearDiff.carrierSpeed * rearRatio;
    return { wheelTorques: torques, pinionSpeed: carrierAvg };
  }

  /**
   * Solve torque split across a single axle differential.
   *
   * Clutch-ramp numbers are treated as engagement strength, not as a literal
   * fraction of driveshaft torque. The previous implementation could generate
   * more locking torque than one wheel's 50% share and reverse that wheel's
   * torque under power, creating a left/right oscillation. Real plate LSDs bias
   * torque strongly, but a normal 1.5-way does not chatter between +drive and
   * -drive on a straight launch.
   */
  private solveAxleDifferential(
    inputTorque: number,
    omegaLeft: number,
    omegaRight: number
  ): {
    torqueLeft: number;
    torqueRight: number;
    carrierSpeed: number;
  } {
    const carrierSpeed = (omegaLeft + omegaRight) * 0.5;
    const deltaOmega = omegaLeft - omegaRight;

    if (this.config.type === 'OPEN') {
      return {
        torqueLeft: inputTorque * 0.5,
        torqueRight: inputTorque * 0.5,
        carrierSpeed,
      };
    }

    if (this.config.type === 'SPOOL') {
      const lockTorque = deltaOmega * 800.0;
      return {
        torqueLeft: inputTorque * 0.5 - lockTorque,
        torqueRight: inputTorque * 0.5 + lockTorque,
        carrierSpeed,
      };
    }

    const isPowerOn = inputTorque >= 0;
    const rampCoeff = PhysicsMath.clamp(isPowerOn ? this.config.powerRamp : this.config.coastRamp, 0, 1);

    // Plate ramp strength is scaled to a realistic axle torque-biasing range.
    // At the default 0.70 power ramp this produces roughly a 70/30 split at
    // strong lock instead of allowing one wheel to receive reverse torque.
    const preload = Math.max(0, this.config.preloadTorque);
    const rampLockCapacity = Math.abs(inputTorque) * rampCoeff * 0.25;
    const maxLockingTorque = preload + rampLockCapacity;

    // Smooth engagement with wheel-speed difference. A gentler slope avoids
    // instant full-lock response to tiny numerical speed differences.
    const lockActivation = Math.tanh(deltaOmega * 0.35);
    let actualLockTorque = maxLockingTorque * lockActivation;

    // Under meaningful drive/coast torque, keep both axle torques in the same
    // direction. Preload is still allowed to cross-couple around zero torque.
    if (Math.abs(inputTorque) > preload * 2) {
      const sameSignCap = Math.abs(inputTorque) * 0.48;
      actualLockTorque = PhysicsMath.clamp(actualLockTorque, -sameSignCap, sameSignCap);
    }

    return {
      torqueLeft: inputTorque * 0.5 - actualLockTorque,
      torqueRight: inputTorque * 0.5 + actualLockTorque,
      carrierSpeed,
    };
  }
}
