from pathlib import Path

# Vehicle: remove duplicate entries from the earlier bootstrap patch and wire
# launch-control state/config into the powertrain.
p = Path('src/physics/Vehicle.ts')
s = p.read_text()
s = s.replace(
"""      reverseRatio: this.config.reverseRatio,
      forwardGearRatios: this.config.forwardGearRatios,
      reverseRatio: this.config.reverseRatio,
      forwardGearRatios: this.config.forwardGearRatios,
""",
"""      reverseRatio: this.config.reverseRatio,
      forwardGearRatios: this.config.forwardGearRatios,
""")
s = s.replace(
"""      maxClutchTorque: this.config.maxClutchTorque,
      transmissionEfficiency: this.config.transmissionEfficiency,
      maxClutchTorque: this.config.maxClutchTorque,
      transmissionEfficiency: this.config.transmissionEfficiency,
""",
"""      maxClutchTorque: this.config.maxClutchTorque,
      transmissionEfficiency: this.config.transmissionEfficiency,
""")
s = s.replace(
"""      reverseRatio: newConfig.reverseRatio,
      forwardGearRatios: newConfig.forwardGearRatios,
      reverseRatio: newConfig.reverseRatio,
      forwardGearRatios: newConfig.forwardGearRatios,
""",
"""      reverseRatio: newConfig.reverseRatio,
      forwardGearRatios: newConfig.forwardGearRatios,
""")
s = s.replace(
"""      maxClutchTorque: newConfig.maxClutchTorque,
      transmissionEfficiency: newConfig.transmissionEfficiency,
      maxClutchTorque: newConfig.maxClutchTorque,
      transmissionEfficiency: newConfig.transmissionEfficiency,
""",
"""      maxClutchTorque: newConfig.maxClutchTorque,
      transmissionEfficiency: newConfig.transmissionEfficiency,
""")
s = s.replace(
"""      (this.wheels[i] as any).radius = newConfig.wheelRadius;
      (this.wheels[i] as any).inertia = newConfig.wheelInertia;
      (this.wheels[i] as any).radius = newConfig.wheelRadius;
      (this.wheels[i] as any).inertia = newConfig.wheelInertia;
""",
"""      (this.wheels[i] as any).radius = newConfig.wheelRadius;
      (this.wheels[i] as any).inertia = newConfig.wheelInertia;
""")

constructor_anchor = """      maxClutchTorque: this.config.maxClutchTorque,
      transmissionEfficiency: this.config.transmissionEfficiency,
      autoBlipDownshift: this.config.autoBlipDownshift,
"""
if 'automaticTorqueConverter: (this.config as any).automaticTorqueConverter' not in s:
    s = s.replace(constructor_anchor, """      maxClutchTorque: this.config.maxClutchTorque,
      transmissionEfficiency: this.config.transmissionEfficiency,
      launchControlEnabled: this.config.launchControlEnabled,
      launchControlRpm: (this.config as any).launchControlRpm,
      lowSpeedTorqueFillNm: (this.config as any).lowSpeedTorqueFillNm,
      torqueFillFadeRpm: (this.config as any).torqueFillFadeRpm,
      automaticTorqueConverter: (this.config as any).automaticTorqueConverter,
      autoBlipDownshift: this.config.autoBlipDownshift,
""")

set_anchor = """      maxClutchTorque: newConfig.maxClutchTorque,
      transmissionEfficiency: newConfig.transmissionEfficiency,
      autoBlipDownshift: newConfig.autoBlipDownshift,
"""
if 'automaticTorqueConverter: (newConfig as any).automaticTorqueConverter' not in s:
    s = s.replace(set_anchor, """      maxClutchTorque: newConfig.maxClutchTorque,
      transmissionEfficiency: newConfig.transmissionEfficiency,
      launchControlEnabled: newConfig.launchControlEnabled,
      launchControlRpm: (newConfig as any).launchControlRpm,
      lowSpeedTorqueFillNm: (newConfig as any).lowSpeedTorqueFillNm,
      torqueFillFadeRpm: (newConfig as any).torqueFillFadeRpm,
      automaticTorqueConverter: (newConfig as any).automaticTorqueConverter,
      autoBlipDownshift: newConfig.autoBlipDownshift,
""")

powertrain_call = """    const powertrainOut = this.powertrain.update(effectiveThrottle, drivenOmega, dt);
"""
if 'this.powertrain.launchControlActive =' not in s:
    s = s.replace(powertrain_call, """    // The G90 M5 can preload its automatic/hybrid powertrain against the brake.
    // Keep this physical state in the powertrain rather than faking extra launch force.
    this.powertrain.launchControlActive = Boolean(
      this.config.launchControlEnabled && inputs.brake > 0.55 && inputs.throttle > 0.80 && speedMs < 2.0
    );
    const powertrainOut = this.powertrain.update(effectiveThrottle, drivenOmega, dt);
""")
p.write_text(s)

# Powertrain: give automatic/hybrid cars torque-converter launch coupling, low-rpm
# e-motor/boost fill, and an actual launch-control staging state.
p = Path('src/physics/Powertrain.ts')
s = p.read_text()
if 'launchControlRpm?: number' not in s:
    s = s.replace(
"""  finalDriveRatio: number; // (e.g. 3.45)
  autoBlipDownshift: boolean;
""",
"""  finalDriveRatio: number; // (e.g. 3.45)
  launchControlEnabled?: boolean;
  launchControlRpm?: number;
  lowSpeedTorqueFillNm?: number;
  torqueFillFadeRpm?: number;
  automaticTorqueConverter?: boolean;
  autoBlipDownshift: boolean;
""")
if 'public launchControlActive: boolean = false;' not in s:
    s = s.replace(
"""  public engineTorqueOutput: number = 0;
""",
"""  public engineTorqueOutput: number = 0;
  public launchControlActive: boolean = false;
""")
if 'this.launchControlActive = false;' not in s:
    s = s.replace(
"""    this.engineTorqueOutput = 0;
""",
"""    this.engineTorqueOutput = 0;
    this.launchControlActive = false;
""", 1)

# Add low-speed hybrid torque fill after the existing turbo multiplier has been applied.
old_torque = """    let rawTorque = this.getRawEngineTorqueCurve(this.engineRpm) * turboBoostMultiplier;
    if (this.isRevLimiting && this.revCutBounce) {
"""
if 'lowSpeedTorqueFillNm' in s and 'const fillFadeRpm' not in s:
    s = s.replace(old_torque, """    let rawTorque = this.getRawEngineTorqueCurve(this.engineRpm) * turboBoostMultiplier;

    // PHEV/EV torque fill: only supplements the low-rpm hole. It fades away well
    // before the upper gears so quarter-mile trap speed still comes from real power.
    const fillTorque = Math.max(0, this.config.lowSpeedTorqueFillNm || 0);
    const fillFadeRpm = Math.max(this.config.idleRpm + 200, this.config.torqueFillFadeRpm || 3200);
    const fillFraction = 1 - PhysicsMath.clamp(
      (this.engineRpm - this.config.idleRpm) / (fillFadeRpm - this.config.idleRpm),
      0,
      1
    );
    rawTorque += fillTorque * fillFraction;

    if (this.isRevLimiting && this.revCutBounce) {
""")

old_capacity = """    let clutchCapacityFraction = 1.0 - this.clutchPedal;
    if (this.isAutomatic || this.gear === 0 || this.gear === 1) {
      if (this.engineRpm < 1400 && this.gear !== 0) {
        const stallMargin = Math.max(0, (this.engineRpm - this.config.idleRpm) / (1400 - this.config.idleRpm));
        clutchCapacityFraction = Math.min(clutchCapacityFraction, stallMargin);
      }
    }
"""
new_capacity = """    let clutchCapacityFraction = 1.0 - this.clutchPedal;
    const hasAutomaticConverter = Boolean(this.isAutomatic && this.config.automaticTorqueConverter);

    if (this.gear === 0) {
      clutchCapacityFraction = 0;
    } else if (this.launchControlActive && this.config.launchControlEnabled && this.gear === 1) {
      // Staging: converter/clutch slips deliberately so the engine and turbos can
      // preload while the service brakes hold the car stationary.
      clutchCapacityFraction = Math.min(clutchCapacityFraction, 0.08);
    } else if (hasAutomaticConverter && this.gear === 1) {
      // A torque-converter automatic transmits meaningful launch torque at low rpm;
      // it does not behave like a manual clutch that transmits zero torque at idle.
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
"""
if old_capacity in s:
    s = s.replace(old_capacity, new_capacity)

# Hold launch-control engine speed near the calibrated staging rpm after flywheel
# integration, while still letting boost/spool/combustion states evolve normally.
engine_update = """    omegaEngine += dOmegaEngine;

    this.engineRpm = Math.max(this.config.idleRpm, (omegaEngine * 30) / Math.PI);
"""
if 'launchTargetRpm' not in s:
    s = s.replace(engine_update, """    omegaEngine += dOmegaEngine;

    if (this.launchControlActive && this.config.launchControlEnabled) {
      const launchTargetRpm = PhysicsMath.clamp(
        this.config.launchControlRpm || 3000,
        this.config.idleRpm,
        this.config.revLimiterRpm - 300
      );
      const launchTargetOmega = (launchTargetRpm * Math.PI) / 30;
      const launchBlend = 1 - Math.exp(-12 * dt);
      omegaEngine += (launchTargetOmega - omegaEngine) * launchBlend;
    }

    this.engineRpm = Math.max(this.config.idleRpm, (omegaEngine * 30) / Math.PI);
""")
p.write_text(s)

# ABS: preserve the correct ~14-15% peak slip target, but use fast reapply and
# a two-stage release so full pedal no longer spends most of a stop at ~40% pressure.
p = Path('src/physics/DriverAids.ts')
s = p.read_text()
start = s.index('  public updateABS(')
end = s.index('  public updateTCS(', start)
new_abs = r'''  public updateABS(
    wheelSlipRatios: [number, number, number, number],
    wheelAngularVelocities: [number, number, number, number],
    speedMs: number,
    isBraking: boolean,
    dt: number
  ): [number, number, number, number] {
    if (this.config.absMode === 'OFF' || !isBraking || speedMs < 1.8) {
      this.absActive = false;
      this.absPressureStates = [1, 1, 1, 1];
      this.absHoldTimers = [0, 0, 0, 0];
      return this.absPressureStates;
    }

    const isSport = this.config.absMode === 'SPORT';
    const targetSlip = isSport ? 0.145 : 0.125;
    const deadband = isSport ? 0.018 : 0.015;
    const minPressure = isSport ? 0.34 : 0.30;
    let anyIntervention = false;

    for (let i = 0; i < 4; i++) {
      const slipMag = Math.max(0, -wheelSlipRatios[i]);
      const nearLock = speedMs > 3.0 && Math.abs(wheelAngularVelocities[i]) < 0.35;
      const effectiveSlip = nearLock ? Math.max(slipMag, 0.9) : slipMag;
      let p = this.absPressureStates[i];

      if (effectiveSlip > 0.34) {
        // A genuinely locking wheel must be released decisively, but only until
        // it recovers; this state should be brief, not the controller's steady state.
        const deepLockRate = isSport ? 7.2 : 8.0;
        p = Math.max(minPressure, p - deepLockRate * dt);
        anyIntervention = true;
      } else if (effectiveSlip > targetSlip + deadband) {
        // Around peak grip, trim pressure gently. The old controller released up
        // to 9 pressure-units/s and averaged ~40% pressure at full pedal.
        const over = effectiveSlip - (targetSlip + deadband);
        const releaseRate = (isSport ? 1.55 : 1.85) + Math.min(1.8, over * 5.0);
        p = Math.max(minPressure, p - releaseRate * dt);
        anyIntervention = true;
      } else if (effectiveSlip < targetSlip - deadband) {
        // Reapply much faster than the modulation release so the caliper spends
        // most of the stop near useful pressure, like a modern four-channel ABS.
        const under = (targetSlip - deadband) - effectiveSlip;
        const reapplyRate = (isSport ? 6.5 : 7.2) + Math.min(3.0, under * 18.0);
        p = Math.min(1.0, p + reapplyRate * dt);
        anyIntervention = anyIntervention || p < 0.995;
      } else {
        anyIntervention = anyIntervention || p < 0.995;
      }

      this.absPressureStates[i] = p;
      this.absHoldTimers[i] = 0;
    }

    this.absActive = anyIntervention;
    return this.absPressureStates;
  }

'''
s = s[:start] + new_abs + s[end:]
p.write_text(s)

# M5-specific automatic/hybrid calibration. These parameters act only in the
# low-rpm launch window and leave the already-correct lateral tire calibration alone.
p = Path('src/physics/m5G90.ts')
s = p.read_text()
if 'launchControlRpm:' not in s:
    s = s.replace("  launchControlEnabled: true,\n", """  launchControlEnabled: true,
  launchControlRpm: 3000,
  lowSpeedTorqueFillNm: 300,
  torqueFillFadeRpm: 3400,
  automaticTorqueConverter: true,
""")
p.write_text(s)
