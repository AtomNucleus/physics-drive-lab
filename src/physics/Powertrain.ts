import { PhysicsMath } from './math/PhysicsMath';

export interface PowertrainConfig {
  maxTorque: number;
  idleRpm: number;
  maxRpm: number;
  revLimiterRpm: number;
  flywheelInertia: number;
  engineBrakingTorque: number;
  clutchBiteRate: number;
  maxClutchTorque?: number;
  transmissionEfficiency?: number;
  turboBoostMaxPsi: number;
  turboSpoolRate: number;
  wastegatePressurePsi: number;
  reverseRatio?: number;
  forwardGearRatios?: number[];
  gearRatios?: number[];
  finalDriveRatio: number;
  launchControlEnabled?: boolean;
  launchControlRpm?: number;
  lowSpeedTorqueFillNm?: number;
  torqueFillFadeRpm?: number;
  automaticTorqueConverter?: boolean;
  shiftDurationSec?: number;
  shiftTorqueMultiplier?: number;
  autoBlipDownshift: boolean;
}

export class Powertrain {
  public config: PowertrainConfig;
  public forwardGearRatios: number[];
  public reverseRatio: number;
  public finalDriveRatio: number;

  public engineRpm: number = 850;
  public flywheelRpm: number = 850;
  public gear: number = 1;
  public isAutomatic: boolean = false;
  public clutchPedal: number = 0;
  public clutchEngaged: boolean = true;
  public isRevLimiting: boolean = false;
  public revCutBounce: boolean = false;
  public turboBoostPsi: number = 0;
  public turboBlowOff: boolean = false;
  public wastegateOpen: boolean = false;
  public deliveredDriveshaftTorque: number = 0;
  public engineTorqueOutput: number = 0;
  public launchControlActive: boolean = false;

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
    this.launchControlActive = false;
    this.shiftTimer = 0;
    this.autoBlipTimer = 0;
    this.revCutTimer = 0;
    this.clutchKickDurationTimer = 0;
    this.prevThrottle = 0;
  }

  public triggerClutchKick() {
    this.clutchPedal = 1.0;
    this.clutchEngaged = false;
    this.clutchKickDurationTimer = 0.20;
  }

  private get shiftDuration(): number {
    return PhysicsMath.clamp(this.config.shiftDurationSec ?? 0.16, 0.025, 0.40);
  }

  public shiftUp() {
    const maxGears = this.forwardGearRatios.length;
    if (this.gear < maxGears && this.shiftTimer <= 0) {
      if (this.gear === -1) this.gear = 0;
      else this.gear++;
      this.shiftTimer = this.shiftDuration;
    }
  }

  public shiftDown() {
    if (this.gear > -1 && this.shiftTimer <= 0) {
      this.gear--;
      this.shiftTimer = this.shiftDuration;
      if (this.config.autoBlipDownshift && this.gear >= 1) this.autoBlipTimer = 0.18;
    }
  }

  public getGearRatio(gear: number): number {
    if (gear === -1) return this.reverseRatio;
    if (gear === 0) return 0;
    const idx = gear - 1;
    if (idx >= 0 && idx < this.forwardGearRatios.length) return this.forwardGearRatios[idx];
    return 1.0;
  }

  public getRawEngineTorqueCurve(rpm: number): number {
    const normRpm = PhysicsMath.clamp(
      (rpm - this.config.idleRpm) / (this.config.maxRpm - this.config.idleRpm),
      0,
      1
    );
    let profile = 0;
    if (normRpm < 0.20) profile = 0.55 + 0.45 * (normRpm / 0.20);
    else if (normRpm <= 0.75) profile = 1.0;
    else profile = 1.0 - 0.25 * ((normRpm - 0.75) / 0.25);
    return this.config.maxTorque * profile;
  }

  public update(
    throttleInput: number,
    drivenAxleAngularVelocity: number,
    dt: number
  ): { engineTorque: number; driveshaftTorque: number; engineRpm: number } {
    if (this.shiftTimer > 0) this.shiftTimer -= dt;
    if (this.autoBlipTimer > 0) this.autoBlipTimer -= dt;
    if (this.clutchKickDurationTimer > 0) {
      this.clutchKickDurationTimer -= dt;
      if (this.clutchKickDurationTimer <= 0) this.clutchPedal = 0;
    }

    let effectiveThrottle = throttleInput;
    if (this.autoBlipTimer > 0) effectiveThrottle = Math.max(effectiveThrottle, 0.75);
    if (this.shiftTimer > 0) {
      effectiveThrottle *= PhysicsMath.clamp(this.config.shiftTorqueMultiplier ?? 0.15, 0, 1);
    }

    if (effectiveThrottle > 0.3) {
      const spoolTarget = (effectiveThrottle * this.config.turboBoostMaxPsi * this.engineRpm) / this.config.maxRpm;
      this.turboBoostPsi += (spoolTarget - this.turboBoostPsi) * Math.min(1.0, this.config.turboSpoolRate * dt);
    } else {
      if (this.prevThrottle > 0.5 && effectiveThrottle < 0.2 && this.turboBoostPsi > 5.0) this.turboBlowOff = true;
      else this.turboBlowOff = false;
      this.turboBoostPsi = Math.max(0, this.turboBoostPsi - 22.0 * dt);
    }
    this.prevThrottle = effectiveThrottle;
    this.wastegateOpen = this.turboBoostPsi >= this.config.wastegatePressurePsi;

    const turboBoostMultiplier = 1.0 + (this.turboBoostPsi / Math.max(1, this.config.turboBoostMaxPsi)) * 0.45;

    if (this.engineRpm >= this.config.revLimiterRpm) {
      this.isRevLimiting = true;
      this.revCutTimer += dt;
      this.revCutBounce = Math.sin(this.revCutTimer * Math.PI * 60) > 0;
    } else {
      this.isRevLimiting = false;
      this.revCutBounce = false;
      this.revCutTimer = 0;
    }

    let rawTorque = this.getRawEngineTorqueCurve(this.engineRpm) * turboBoostMultiplier;
    const fillTorque = Math.max(0, this.config.lowSpeedTorqueFillNm || 0);
    const fillFadeRpm = Math.max(this.config.idleRpm + 200, this.config.torqueFillFadeRpm || 3200);
    const fillFraction = 1 - PhysicsMath.clamp(
      (this.engineRpm - this.config.idleRpm) / (fillFadeRpm - this.config.idleRpm),
      0,
      1
    );
    rawTorque += fillTorque * fillFraction;

    if (this.isRevLimiting && this.revCutBounce) rawTorque = 0;

    let engineCombustionTorque = 0;
    if (effectiveThrottle > 0.02) engineCombustionTorque = rawTorque * effectiveThrottle;
    else {
      const brakingFactor = this.engineRpm / this.config.maxRpm;
      engineCombustionTorque = -this.config.engineBrakingTorque * brakingFactor;
    }
    this.engineTorqueOutput = engineCombustionTorque;

    const currentGearRatio = this.getGearRatio(this.gear);
    const finalDrive = this.finalDriveRatio;
    const totalGearRatio = currentGearRatio * finalDrive;
    const clutchPlateAngularVelocity = drivenAxleAngularVelocity * finalDrive * currentGearRatio;

    let omegaEngine = (this.engineRpm * Math.PI) / 30;
    let clutchCapacityFraction = 1.0 - this.clutchPedal;
    const hasAutomaticConverter = Boolean(this.isAutomatic && this.config.automaticTorqueConverter);

    if (this.gear === 0) {
      clutchCapacityFraction = 0;
    } else if (this.launchControlActive && this.config.launchControlEnabled && this.gear === 1) {
      clutchCapacityFraction = Math.min(clutchCapacityFraction, 0.08);
    } else if (hasAutomaticConverter && this.gear === 1) {
      const converterCoupling = PhysicsMath.clamp(
        0.42 + 0.58 * ((this.engineRpm - this.config.idleRpm) / 1800),
        0.42,
        1.0
      );
      clutchCapacityFraction = Math.min(clutchCapacityFraction, converterCoupling);
    } else if (this.gear === 1 && this.engineRpm < 1400) {
      const stallMargin = Math.max(0, (this.engineRpm - this.config.idleRpm) / (1400 - this.config.idleRpm));
      clutchCapacityFraction = Math.min(clutchCapacityFraction, stallMargin);
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
      transmittedClutchTorque = PhysicsMath.clamp(clutchSlipTorque, -maxClutchTorqueCapacity, maxClutchTorqueCapacity);
      this.clutchEngaged = Math.abs(deltaOmega) < 1.5;
    }

    const netFlywheelTorque = engineCombustionTorque - transmittedClutchTorque;
    const flywheelInertia = Math.max(0.08, this.config.flywheelInertia);
    omegaEngine += (netFlywheelTorque / flywheelInertia) * dt;

    if (this.launchControlActive && this.config.launchControlEnabled) {
      const launchTargetRpm = PhysicsMath.clamp(
        this.config.launchControlRpm || 3000,
        this.config.idleRpm,
        this.config.revLimiterRpm - 300
      );
      omegaEngine = (launchTargetRpm * Math.PI) / 30;
    }

    this.engineRpm = Math.max(this.config.idleRpm, (omegaEngine * 30) / Math.PI);
    this.flywheelRpm = this.engineRpm;
    if (this.engineRpm > this.config.maxRpm) {
      this.engineRpm = this.config.maxRpm;
      omegaEngine = (this.engineRpm * Math.PI) / 30;
    }

    const transmissionEfficiency = this.config.transmissionEfficiency || 0.95;
    this.deliveredDriveshaftTorque = transmittedClutchTorque * totalGearRatio * transmissionEfficiency;

    return {
      engineTorque: this.engineTorqueOutput,
      driveshaftTorque: this.deliveredDriveshaftTorque,
      engineRpm: this.engineRpm,
    };
  }
}
