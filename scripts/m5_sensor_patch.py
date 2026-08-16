from pathlib import Path

# Driver aids should sense instantaneous wheel-speed slip. Tire forces retain their
# relaxation state, so electronics react quickly without making the contact patch instant.
p = Path('src/physics/Vehicle.ts')
s = p.read_text()
s = s.replace(
"""        ? [this.wheels[0].relaxationSlipRatio, this.wheels[1].relaxationSlipRatio]
        : this.config.drivetrain === 'RWD'
        ? [this.wheels[2].relaxationSlipRatio, this.wheels[3].relaxationSlipRatio]
        : this.wheels.map((w) => w.relaxationSlipRatio);
""",
"""        ? [this.wheels[0].rawSlipRatio, this.wheels[1].rawSlipRatio]
        : this.config.drivetrain === 'RWD'
        ? [this.wheels[2].rawSlipRatio, this.wheels[3].rawSlipRatio]
        : this.wheels.map((w) => w.rawSlipRatio);
""")
s = s.replace(
"""    const wheelSlips: [number, number, number, number] = [
      this.wheels[0].relaxationSlipRatio,
      this.wheels[1].relaxationSlipRatio,
      this.wheels[2].relaxationSlipRatio,
      this.wheels[3].relaxationSlipRatio,
    ];
""",
"""    const wheelSlips: [number, number, number, number] = [
      this.wheels[0].rawSlipRatio,
      this.wheels[1].rawSlipRatio,
      this.wheels[2].rawSlipRatio,
      this.wheels[3].rawSlipRatio,
    ];
""")
p.write_text(s)

# Launch control is an engine-speed hold, not a soft spring toward target rpm.
# The previous soft blend allowed combustion torque to overpower the governor and
# staged at ~5,180 rpm despite a 3,000-rpm request.
p = Path('src/physics/Powertrain.ts')
s = p.read_text()
old = """      const launchTargetOmega = (launchTargetRpm * Math.PI) / 30;
      const launchBlend = 1 - Math.exp(-12 * dt);
      omegaEngine += (launchTargetOmega - omegaEngine) * launchBlend;
"""
new = """      const launchTargetOmega = (launchTargetRpm * Math.PI) / 30;
      omegaEngine = launchTargetOmega;
"""
s = s.replace(old, new)
p.write_text(s)
