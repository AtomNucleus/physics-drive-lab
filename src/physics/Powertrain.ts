import { PhysicsMath } from './math/PhysicsMath';

export interface PowertrainConfig {
  maxTorque: number; // Nm (e.g. 540)
  idleRpm: number; // e.g. 850
  maxRpm: number; // e.g. 7600
  revLimiterRpm: number; // e.g. 7450
  flywheelInertia: number; // kg*m^2 (e.g. 0.18)
  engineBrakingTorque: number; // Nm (e.g. 85)
  clutchBiteRate: number; // s^-1 (e.g. 14.0)
  maxClutchTorque?: number; // Nm (e.g. 1200)
  transmissionEfficiency?: number; // (e.g. 0.95)
  turboBoostMaxPsi: number; // PSI (e.g. 18.0)
  turboSpoolRate: number; // s^-1 (e.g. 3.5)
  wastegatePressurePsi: number;
  reverseRatio?: number; // (e.g. -3.40)
  forwardGearRatios?: number[]; // [3.82, 2.36, 1.68, 1.29, 1.00, 0.79]
  gearRatios?: number[]; // Legacy array [R, 1, 2, 3, 4, 5, 6]
  finalDriveRatio: number; // (e.g. 3.45)
  autoBlipDownshift: boolean;
}

export class Powertrain {
  public config: PowertrainConfig;
  public forwardGearRatios: number[];
  public reverseRatio: number;
  public finalDriveRatio: number;

  // States
  public engineRpm: number = 850;
  public flywheelRpm: number = 850;
  public gear: number = 1; // -1 = R, 0 = N, 1..6
  public isAutomatic: boolean = false;
  public clutchPedal: number = 0; // 0 = fully closed/engaged, 1 = fully disengaged
  public clutchEngaged: boolean = true;
  public isRevLimiting: boolean = false;
  public revCutBounce: boolean = false;
  public turboBoostPsi: number = 0;
  public turboBlowOff: boolean = false;
  public wastegateOpen: boolean = false;
  public deliveredDriveshaftTorque: number = 0;
  public engineTorqueOutput: number = 0;

  // Timers & Internal State
  private shiftTimer: number = 0;
  private autoBlipTimer: number = 0;
  private revCutTimer: number = 0;
  private clutchKickDurationTimer: number = 0;
  private prevThrottle: number = 0;

  constructor(config: PowertrainConfig) {
    this.config = { ...config };
    this.finalDriveRatio = config.finalDriveRatio || 3.45;
    this.reverseRatio = config.reverseRatio !== undefined ? config.reverseRatio : (config.gearRatios ? config.gearRatios[0] : -3.40);
    this.forwardGearRatios = config.forwardGearRatios || (config.gearRatios ? config.gearRatios.slice(1) : [3.82, 2.36, 1.68, 1.29, 1.00, 0.79]);
    this.engineRpm = config.idleRpm;
    this.flywheelRpm = config.idleRpm;
  }

  public reset() {
    this.engineRpm = this.config.idleRpm;
    this.flywheelRpm = this.config.idleRpm;
    this.gear = 1;
    this.clutchPedal = 0;
    this.clutchEngaged = true;
    this.isRevLimiting = false;
    this.revCutBounce = false;
    this.turboBoostPsi = 0;
    this.turboBlowOff = false;
    this.wastegateOpen = false;
    this.deliveredDriveshaftTorque = 0;
    this.engineTorqueOutput = 0;
    this.shiftTimer = 0;
    this.autoBlipTimer = 0;
    this.revCutTimer = 0;
    this.clutchKickDurationTimer = 0;
    this.prevThrottle = 0;
  }

  public triggerClutchKick() {
    this.clutchPedal = 1.0;
    this.clutchEngaged = false;
    this.clutchKickDurationTimer = 0.20; // 200 ms
  }

  public shiftUp() {
    const maxGears = this.forwardGearRatios.length;
    if (this.gear < maxGears && this.shiftTimer <= 0) {
      if (this.gear === -1) {
        this.gear = 0;
      } else {
        this.gear++;
      }
      this.shiftTimer = 0.16; // 160ms shift interruption
    }
  }

  public shiftDown() {
    if (this.gear > -1 && this.shiftTimer <= 0) {
      this.gear--;
      this.shiftTimer = 0.16;
      if (this.config.autoBlipDownshift && this.gear >= 1) {
        this.autoBlipTimer = 0.18;
      }
    }
  }

  public getGearRatio(gear: number): number {
    if (gear === -1) {
      return this.reverseRatio;
    }
    if (gear === 0) {
      return 0; // Neutral
    }
    const idx = gear - 1;
    if (idx >= 0 && idx < this.forwardGearRatios.length) {
      return this.forwardGearRatios[idx];
    }
    return 1.0;
  }

  /**
   * Sample engine torque curve from RPM
   */
  public getRawEngineTorqueCurve(rpm: number): number {
    const normRpm = PhysicsMath.clamp(
      (rpm - this.config.idleRpm) / (this.config.maxRpm - this.config.idleRpm),
      0,
      1
    );

    let profile = 0;
    if (normRpm < 0.20) {
      profile = 0.55 + 0.45 * (normRpm / 0.20);
    } else if (normRpm <= 0.75) {
      profile = 1.0;
    } else {
      profile = 1.0 - 0.25 * ((normRpm - 0.75) / 0.25);
    }

    return this.config.maxTorque * profile;
  }

  /**
   * Step the powertrain simulation
   *
   * @param throttleInput 0 to 1
   * @param drivenAxleAngularVelocity Angular velocity of driven axle wheels (rad/s)
   * @param dt Timestep (s)
   */
  public update(
    throttleInput: number,
    drivenAxleAngularVelocity: number,
    dt: number
  ): {
    engineTorque: number;
    driveshaftTorque: number;
    engineRpm: number;
  } {
    // 1. Shift Timers & Auto-Blip
    if (this.shiftTimer > 0) {
      this.shiftTimer -= dt;
    }

    if (this.autoBlipTimer > 0) {
      this.autoBlipTimer -= dt;
    }

    if (this.clutchKickDurationTimer > 0) {
      this.clutchKickDurationTimer -= dt;
      if (this.clutchKickDurationTimer <= 0) {
        this.clutchPedal = 0; // Dump clutch back in
      }
    }

    // Effective throttle command
    let effectiveThrottle = throttleInput;
    if (this.autoBlipTimer > 0) {
      effectiveThrottle = Math.max(effectiveThrottle, 0.75); // Rev-match blip
    }
    if (this.shiftTimer > 0) {
      effectiveThrottle *= 0.15; // Cut ignition torque during shift
    }

    // 2. Turbocharger Spool Dynamics
    if (effectiveThrottle > 0.3) {
      const spoolTarget = (effectiveThrottle * this.config.turboBoostMaxPsi * this.engineRpm) / this.config.maxRpm;
      this.turboBoostPsi += (spoolTarget - this.turboBoostPsi) * Math.min(1.0, this.config.turboSpoolRate * dt);
    } else {
      if (this.prevThrottle > 0.5 && effectiveThrottle < 0.2 && this.turboBoostPsi > 5.0) {
        this.turboBlowOff = true;
      } else {
        this.turboBlowOff = false;
      }
      this.turboBoostPsi = Math.max(0, this.turboBoostPsi - 22.0 * dt);
    }
    this.prevThrottle = effectiveThrottle;
    this.wastegateOpen = this.turboBoostPsi >= this.config.wastegatePressurePsi;

    // Turbo boost torque multiplier
    const turboBoostMultiplier = 1.0 + (this.turboBoostPsi / Math.max(1, this.config.turboBoostMaxPsi)) * 0.45;

    // 3. Engine Rev Limiter Spark/Fuel Cut
    if (this.engineRpm >= this.config.revLimiterRpm) {
      this.isRevLimiting = true;
      this.revCutTimer += dt;
      this.revCutBounce = Math.sin(this.revCutTimer * Math.PI * 60) > 0;
    } else {
      this.isRevLimiting = false;
      this.revCutBounce = false;
      this.revCutTimer = 0;
    }

    // 4. Engine Torque Calculation
    let rawTorque = this.getRawEngineTorqueCurve(this.engineRpm) * turboBoostMultiplier;
    if (this.isRevLimiting && this.revCutBounce) {
      rawTorque = 0; // Cut cylinder ignition
    }

    let engineCombustionTorque = 0;
    if (effectiveThrottle > 0.02) {
      engineCombustionTorque = rawTorque * effectiveThrottle;
    } else {
      const brakingFactor = this.engineRpm / this.config.maxRpm;
      engineCombustionTorque = -this.config.engineBrakingTorque * brakingFactor;
    }
    this.engineTorqueOutput = engineCombustionTorque;

    // 5. Gearbox & Transmission Kinematics
    const currentGearRatio = this.getGearRatio(this.gear);
    const finalDrive = this.finalDriveRatio;
    const totalGearRatio = currentGearRatio * finalDrive;

    // Pinion speed = drivenAxleAngularVelocity * finalDriveRatio
    // Clutch plate input angular velocity = pinionSpeed * currentGearRatio = drivenAxleAngularVelocity * finalDriveRatio * currentGearRatio
    const clutchPlateAngularVelocity = drivenAxleAngularVelocity * finalDrive * currentGearRatio;

    // 6. Clutch Dynamics & Torque Transmission
    let omegaEngine = (this.engineRpm * Math.PI) / 30;

    let clutchCapacityFraction = 1.0 - this.clutchPedal;
    if (this.isAutomatic || this.gear === 0 || this.gear === 1) {
      if (this.engineRpm < 1400 && this.gear !== 0) {
        const stallMargin = Math.max(0, (this.engineRpm - this.config.idleRpm) / (1400 - this.config.idleRpm));
        clutchCapacityFraction = Math.min(clutchCapacityFraction, stallMargin);
      }
    }

    const baseClutchCapacity = this.config.maxClutchTorque || (this.config.maxTorque * 2.2);
    const maxClutchTorqueCapacity = baseClutchCapacity * clutchCapacityFraction;

    let transmittedClutchTorque = 0;

    if (this.gear === 0 || clutchCapacityFraction < 0.05) {
      this.clutchEngaged = false;
      transmittedClutchTorque = 0;
    } else {
      const deltaOmega = omegaEngine - clutchPlateAngularVelocity;
      const clutchSlipTorque = deltaOmega * 95.0;

      transmittedClutchTorque = PhysicsMath.clamp(
        clutchSlipTorque,
        -maxClutchTorqueCapacity,
        maxClutchTorqueCapacity
      );

      this.clutchEngaged = Math.abs(deltaOmega) < 1.5;
    }

    // 7. Engine Flywheel Rotational Acceleration (I_e * d_omega/dt = T_combustion - T_clutch)
    const netFlywheelTorque = engineCombustionTorque - transmittedClutchTorque;
    const flywheelInertia = Math.max(0.08, this.config.flywheelInertia);
    const dOmegaEngine = (netFlywheelTorque / flywheelInertia) * dt;

    omegaEngine += dOmegaEngine;

    this.engineRpm = Math.max(this.config.idleRpm, (omegaEngine * 30) / Math.PI);
    this.flywheelRpm = this.engineRpm;

    if (this.engineRpm > this.config.maxRpm) {
      this.engineRpm = this.config.maxRpm;
      omegaEngine = (this.engineRpm * Math.PI) / 30;
    }

    // 8. Delivered Torque to Driven Axle (conserving mechanical power)
    const transmissionEfficiency = this.config.transmissionEfficiency || 0.95;
    this.deliveredDriveshaftTorque =
      transmittedClutchTorque * totalGearRatio * transmissionEfficiency;

    return {
      engineTorque: this.engineTorqueOutput,
      driveshaftTorque: this.deliveredDriveshaftTorque,
      engineRpm: this.engineRpm,
    };
  }
}
