from pathlib import Path

# Remove the experimental post-clutch launch torque cap. The physically correct
# fix is to give driven wheels the rotational inertia reflected through the active gear.
p = Path('src/physics/Vehicle.ts')
s = p.read_text()
start_marker = """    let commandedDriveshaftTorque = powertrainOut.driveshaftTorque;

    // Launch control is a wheel-torque controller, not simply a rev hold plus TCS.
"""
end_marker = """    const diffOut = this.differential.distributeTorque(commandedDriveshaftTorque, wheelOmegas);
"""
if start_marker in s and end_marker in s:
    start = s.index(start_marker)
    end = s.index(end_marker, start) + len(end_marker)
    s = s[:start] + "    const diffOut = this.differential.distributeTorque(powertrainOut.driveshaftTorque, wheelOmegas);\n" + s[end:]

# Compute source inertia reflected through the current gear/final drive. The G90's
# V8 crank/flywheel, torque-converter turbine, e-motor rotor and gearbox input train
# all resist instantaneous wheel-speed changes; bare wheel inertia alone cannot.
anchor = """    const brakeTorques = this.brakes.calculateBrakeTorques(inputs.brake, inputs.handbrake);

    // 7. Solve 4 Wheels & Apply Contact Forces to Rigid Body
"""
if 'reflectedDrivelineInertiaPerDrivenWheel' not in s:
    repl = """    const brakeTorques = this.brakes.calculateBrakeTorques(inputs.brake, inputs.handbrake);

    const currentGear = this.powertrain.gear;
    const currentGearRatio = currentGear > 0
      ? Math.abs(this.config.forwardGearRatios[currentGear - 1] ?? this.config.gearRatios[currentGear] ?? 0)
      : 0;
    const totalRatio = currentGearRatio * Math.abs(this.config.finalDriveRatio);
    const drivenWheelCount = this.config.drivetrain === 'AWD' ? 4 : 2;
    const drivelineInputInertia = Math.max(
      0,
      (this.config as any).drivelineInputInertia ?? this.config.flywheelInertia
    );
    const drivelineCoupling = PhysicsMath.clamp(
      (this.config as any).drivelineInertiaCoupling ?? 0.75,
      0,
      1.5
    );
    const reflectedDrivelineInertiaPerDrivenWheel =
      drivenWheelCount > 0
        ? (drivelineInputInertia * totalRatio * totalRatio * drivelineCoupling) / drivenWheelCount
        : 0;

    // 7. Solve 4 Wheels & Apply Contact Forces to Rigid Body
"""
    s = s.replace(anchor, repl)

# Pass reflected inertia only to driven wheels.
old_call_tail = """        surface.friction * this.config.ambientSurfaceFrictionMultiplier,
        surface.rollingResistance,
        dt
      );
"""
if 'reflectedInertiaForWheel' not in s:
    new_call_tail = """        surface.friction * this.config.ambientSurfaceFrictionMultiplier,
        surface.rollingResistance,
        dt,
        (() => {
          const isDriven =
            this.config.drivetrain === 'AWD' ||
            (this.config.drivetrain === 'FWD' && i < 2) ||
            (this.config.drivetrain === 'RWD' && i >= 2);
          return isDriven ? reflectedDrivelineInertiaPerDrivenWheel : 0;
        })()
      );
"""
    s = s.replace(old_call_tail, new_call_tail)
p.write_text(s)

# Wheel dynamics: include gear-reflected driveline inertia in the rotational DOF.
p = Path('src/physics/WheelDynamics.ts')
s = p.read_text()
old_sig = """    rollingResistance: number,
    dt: number
  ): TireForceOutput {
"""
new_sig = """    rollingResistance: number,
    dt: number,
    reflectedDrivelineInertia: number = 0
  ): TireForceOutput {
"""
s = s.replace(old_sig, new_sig)
old_accel = """    const angularAccel = PhysicsMath.clamp(
      (driveTorque - brakeTorque - tireReactionTorque) / this.inertia,
      -4500,
      4500
    );
"""
new_accel = """    const effectiveRotationalInertia = this.inertia + Math.max(0, reflectedDrivelineInertia);
    const angularAccel = PhysicsMath.clamp(
      (driveTorque - brakeTorque - tireReactionTorque) / effectiveRotationalInertia,
      -4500,
      4500
    );
"""
s = s.replace(old_accel, new_accel)
p.write_text(s)

# M5 input-side inertia includes V8/flywheel, converter turbine, e-motor and gearbox
# input components. BMW does not publish this; it is an explicit calibration estimate.
p = Path('src/physics/m5G90.ts')
s = p.read_text()
if 'drivelineInputInertia:' not in s:
    s = s.replace(
        "  automaticTorqueConverter: true,\n",
        """  automaticTorqueConverter: true,
  drivelineInputInertia: 0.55,
  drivelineInertiaCoupling: 1.0,
"""
    )
p.write_text(s)
