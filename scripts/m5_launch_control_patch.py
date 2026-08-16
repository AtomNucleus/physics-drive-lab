from pathlib import Path

p = Path('src/physics/Vehicle.ts')
s = p.read_text()
old = """    const diffOut = this.differential.distributeTorque(powertrainOut.driveshaftTorque, wheelOmegas);
"""
new = """    let commandedDriveshaftTorque = powertrainOut.driveshaftTorque;

    // Launch control is a wheel-torque controller, not simply a rev hold plus TCS.
    // The available tire force from the previous 120 Hz contact solve provides a
    // feed-forward traction ceiling; raw wheel-speed slip then trims that ceiling.
    const launchTorqueControlActive = Boolean(
      this.config.launchControlEnabled &&
      inputs.throttle > 0.80 &&
      inputs.brake < 0.10 &&
      speedMs < ((this.config as any).launchControlEndSpeedMs ?? 30.0)
    );
    if (launchTorqueControlActive && commandedDriveshaftTorque > 0) {
      const targetSlip = (this.config as any).launchSlipTarget ?? 0.12;
      const utilization = PhysicsMath.clamp((this.config as any).launchTractionUtilization ?? 0.96, 0.70, 1.05);
      const maxPositiveSlip = Math.max(0, ...drivenSlips.map((slip) => Math.max(0, slip)));
      const slipError = Math.max(0, maxPositiveSlip - targetSlip);
      const slipTrim = PhysicsMath.clamp(1.0 - slipError * 1.65, 0.22, 1.0);

      let tireTorqueCapacity = 0;
      for (let i = 0; i < 4; i++) {
        const tireLimit = Math.max(
          0,
          this.wheels[i].lastTireOutput?.frictionLimit ??
          (this.suspension.states[i].forceNorm * (i < 2 ? this.config.tireGripFront : this.config.tireGripRear))
        );
        tireTorqueCapacity += tireLimit * this.config.wheelRadius;
      }

      const launchTorqueCeiling = tireTorqueCapacity * utilization * slipTrim;
      commandedDriveshaftTorque = Math.min(commandedDriveshaftTorque, launchTorqueCeiling);
    }

    const diffOut = this.differential.distributeTorque(commandedDriveshaftTorque, wheelOmegas);
"""
if old in s:
    s = s.replace(old, new)
p.write_text(s)

p = Path('src/physics/m5G90.ts')
s = p.read_text()
if 'launchSlipTarget:' not in s:
    s = s.replace(
        "  automaticTorqueConverter: true,\n",
        """  automaticTorqueConverter: true,
  launchSlipTarget: 0.12,
  launchTractionUtilization: 0.98,
  launchControlEndSpeedMs: 30.0,
"""
    )
p.write_text(s)
