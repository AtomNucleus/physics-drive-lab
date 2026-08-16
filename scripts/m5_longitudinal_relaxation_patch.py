from pathlib import Path

# Tire config: steering/cornering and longitudinal slip do not need to share one
# relaxation length. Keep the longer lateral carcass response for weight/steering feel,
# while allowing drive/brake force to build over a shorter belt/contact-patch length.
p = Path('src/physics/TireModel.ts')
s = p.read_text()
if 'longitudinalRelaxationLength?: number;' not in s:
    s = s.replace(
        '  relaxationLength: number;\n',
        '  relaxationLength: number;\n  longitudinalRelaxationLength?: number;\n  longitudinalForceRelaxationLength?: number;\n'
    )
p.write_text(s)

p = Path('src/physics/Vehicle.ts')
s = p.read_text()
s = s.replace(
    '      relaxationLength: this.config.relaxationLength,\n      pneumaticTrailMax:',
    '      relaxationLength: this.config.relaxationLength,\n      longitudinalRelaxationLength: (this.config as any).longitudinalRelaxationLength,\n      longitudinalForceRelaxationLength: (this.config as any).longitudinalForceRelaxationLength,\n      pneumaticTrailMax:'
)
s = s.replace(
    '        relaxationLength: newConfig.relaxationLength,\n        pneumaticTrailMax:',
    '        relaxationLength: newConfig.relaxationLength,\n        longitudinalRelaxationLength: (newConfig as any).longitudinalRelaxationLength,\n        longitudinalForceRelaxationLength: (newConfig as any).longitudinalForceRelaxationLength,\n        pneumaticTrailMax:'
)
p.write_text(s)

p = Path('src/physics/WheelDynamics.ts')
s = p.read_text()
old_slip = """      const sigma = Math.max(0.035, this.tireConfig.relaxationLength);
      const relaxationTravel = Math.max(2.0, Math.abs(longitudinalVelocity)) * dt;
      const slipAlpha = 1 - Math.exp(-relaxationTravel / sigma);
      this.relaxationSlipAngle += (this.rawSlipAngle - this.relaxationSlipAngle) * slipAlpha;
      this.relaxationSlipRatio += (this.rawSlipRatio - this.relaxationSlipRatio) * slipAlpha;
"""
new_slip = """      const lateralSigma = Math.max(0.035, this.tireConfig.relaxationLength);
      const longitudinalSigma = Math.max(
        0.025,
        this.tireConfig.longitudinalRelaxationLength ?? this.tireConfig.relaxationLength
      );
      const relaxationTravel = Math.max(2.0, Math.abs(longitudinalVelocity)) * dt;
      const lateralSlipAlpha = 1 - Math.exp(-relaxationTravel / lateralSigma);
      const longitudinalSlipAlpha = 1 - Math.exp(-relaxationTravel / longitudinalSigma);
      this.relaxationSlipAngle += (this.rawSlipAngle - this.relaxationSlipAngle) * lateralSlipAlpha;
      this.relaxationSlipRatio += (this.rawSlipRatio - this.relaxationSlipRatio) * longitudinalSlipAlpha;
"""
s = s.replace(old_slip, new_slip)

old_force = """    const sigmaForce = Math.max(0.025, this.tireConfig.relaxationLength * 0.55);
    const forceTravel = Math.max(2.5, Math.abs(longitudinalVelocity)) * dt;
    const forceAlpha = 1 - Math.exp(-forceTravel / sigmaForce);
    this.transientFx += (target.fx - this.transientFx) * forceAlpha;
    this.transientFy += (target.fy - this.transientFy) * forceAlpha;
    this.transientMz += (target.aligningTorque - this.transientMz) * forceAlpha;
"""
new_force = """    const lateralForceSigma = Math.max(0.025, this.tireConfig.relaxationLength * 0.55);
    const longitudinalForceSigma = Math.max(
      0.018,
      this.tireConfig.longitudinalForceRelaxationLength ??
        ((this.tireConfig.longitudinalRelaxationLength ?? this.tireConfig.relaxationLength) * 0.55)
    );
    const forceTravel = Math.max(2.5, Math.abs(longitudinalVelocity)) * dt;
    const lateralForceAlpha = 1 - Math.exp(-forceTravel / lateralForceSigma);
    const longitudinalForceAlpha = 1 - Math.exp(-forceTravel / longitudinalForceSigma);
    this.transientFx += (target.fx - this.transientFx) * longitudinalForceAlpha;
    this.transientFy += (target.fy - this.transientFy) * lateralForceAlpha;
    this.transientMz += (target.aligningTorque - this.transientMz) * lateralForceAlpha;
"""
s = s.replace(old_force, new_force)
p.write_text(s)

p = Path('src/physics/m5G90.ts')
s = p.read_text()
if 'longitudinalRelaxationLength:' not in s:
    s = s.replace(
        '  relaxationLength: 0.19,\n',
        '  relaxationLength: 0.19,\n  longitudinalRelaxationLength: 0.08,\n  longitudinalForceRelaxationLength: 0.045,\n'
    )
p.write_text(s)
