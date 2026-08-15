import { DifferentialType, DrivetrainType } from '../types';
import { PhysicsMath } from './math/PhysicsMath';

export interface DifferentialConfig {
  type: DifferentialType;
  powerRamp: number; // 0..1 (e.g. 0.70)
  coastRamp: number; // 0..1 (e.g. 0.35)
  preloadTorque: number; // Nm (e.g. 45)
  drivetrain: DrivetrainType; // 'RWD' | 'FWD' | 'AWD'
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
      // Front-wheel drive
      const diffOut = this.solveAxleDifferential(inputTorque, omegaFL, omegaFR);
      torques[0] = diffOut.torqueLeft;
      torques[1] = diffOut.torqueRight;
      torques[2] = 0;
      torques[3] = 0;
      return { wheelTorques: torques, pinionSpeed: diffOut.carrierSpeed };
    }

    if (this.config.drivetrain === 'RWD') {
      // Rear-wheel drive
      const diffOut = this.solveAxleDifferential(inputTorque, omegaRL, omegaRR);
      torques[0] = 0;
      torques[1] = 0;
      torques[2] = diffOut.torqueLeft;
      torques[3] = diffOut.torqueRight;
      return { wheelTorques: torques, pinionSpeed: diffOut.carrierSpeed };
    }

    // AWD: Center Differential with 40/60 Front/Rear torque split
    const frontRatio = 0.40;
    const rearRatio = 0.60;

    const torqueFrontAxle = inputTorque * frontRatio;
    const torqueRearAxle = inputTorque * rearRatio;

    const frontDiff = this.solveAxleDifferential(torqueFrontAxle, omegaFL, omegaFR);
    const rearDiff = this.solveAxleDifferential(torqueRearAxle, omegaRL, omegaRR);

    torques[0] = frontDiff.torqueLeft;
    torques[1] = frontDiff.torqueRight;
    torques[2] = rearDiff.torqueLeft;
    torques[3] = rearDiff.torqueRight;

    const carrierAvg = frontDiff.carrierSpeed * frontRatio + rearDiff.carrierSpeed * rearRatio;
    return { wheelTorques: torques, pinionSpeed: carrierAvg };
  }

  /**
   * Solve torque split across a single axle differential
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
    const deltaOmega = omegaLeft - omegaRight; // Positive if left wheel is spinning faster

    if (this.config.type === 'OPEN') {
      // True open differential: 50/50 torque split regardless of wheel speed difference
      return {
        torqueLeft: inputTorque * 0.5,
        torqueRight: inputTorque * 0.5,
        carrierSpeed,
      };
    }

    if (this.config.type === 'SPOOL') {
      // Fully locked spool axle: equal speed, torque biases heavily to slower wheel
      const lockTorque = deltaOmega * 800.0;
      return {
        torqueLeft: inputTorque * 0.5 - lockTorque,
        torqueRight: inputTorque * 0.5 + lockTorque,
        carrierSpeed,
      };
    }

    // Clutch-Type LSD (1.5-way / 2-way)
    const isPowerOn = inputTorque >= 0;
    const rampCoeff = isPowerOn ? this.config.powerRamp : this.config.coastRamp;

    // Locking torque capacity = Preload + Ramp * |Input Torque|
    const maxLockingTorque = this.config.preloadTorque + rampCoeff * Math.abs(inputTorque);

    // Dynamic friction engagement curve as a function of speed difference
    // Uses tanh for smooth, non-oscillating transition through zero delta-speed
    const lockActivation = Math.tanh(deltaOmega * 1.5);
    const actualLockTorque = maxLockingTorque * lockActivation;

    // Faster wheel receives less torque, slower wheel receives more torque
    const torqueLeft = inputTorque * 0.5 - actualLockTorque;
    const torqueRight = inputTorque * 0.5 + actualLockTorque;

    return {
      torqueLeft,
      torqueRight,
      carrierSpeed,
    };
  }
}
