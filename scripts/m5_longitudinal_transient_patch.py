from pathlib import Path

# Tire model config: allow longitudinal slip/force transients to differ from
# lateral carcass response. Real tires do not have to share one relaxation length
# for both force directions, and keeping them separate lets launch/braking response
# be calibrated without making steering response artificially sharp.
p = Path('src/physics/TireModel.ts')
s = p.read_text()
if 'longitudinalRelaxationLength?: number;' not in s:
    s = s.replace(
        '  relaxationLength: number;\n',
        '  relaxationLength: number;\n  longitudinalRelaxationLength?: number;\n  longitudinalForceRelaxationLength?: number;\n'
    )
p.write_text(s)

# Wheel dynamics: retain the existing lateral relaxation path, but give slip ratio
# and Fx their own travel-domain time constants.
p = Path('src/physics/WheelDynamics.ts')
s = p.read_text()
old_relax = '''      const sigma = Math.max(0.035, this.tireConfig.relaxationLength);
      const relaxationTravel = Math.max(2.0, Math.abs(longitudinalVelocity)) * dt;
      const slipAlpha = 1 - Math.exp(-relaxationTravel / sigma);
      this.relaxationSlipAngle += (this.rawSlipAngle - this.relaxationSlipAngle) * slipAlpha;
      this.relaxationSlipRatio += (this.rawSlipRatio - this.relaxationSlipRatio) * slipAlpha;
'''
new_relax = '''      const lateralSigma = Math.max(0.035, this.tireConfig.relaxationLength);
      const longitudinalSigma = Math.max(
        0.025,
        this.tireConfig.longitudinalRelaxationLength ?? lateralSigma
      );
      const relaxationTravel = Math.max(2.0, Math.abs(longitudinalVelocity)) * dt;
      const lateralSlipAlpha = 1 - Math.exp(-relaxationTravel / lateralSigma);
      const longitudinalSlipAlpha = 1 - Math.exp(-relaxationTravel / longitudinalSigma);
      this.relaxationSlipAngle += (this.rawSlipAngle - this.relaxationSlipAngle) * lateralSlipAlpha;
      this.relaxationSlipRatio += (this.rawSlipRatio - this.relaxationSlipRatio) * longitudinalSlipAlpha;
'''
if old_relax in s:
    s = s.replace(old_relax, new_relax)

old_force = '''    const sigmaForce = Math.max(0.025, this.tireConfig.relaxationLength * 0.55);
    const forceTravel = Math.max(2.5, Math.abs(longitudinalVelocity)) * dt;
    const forceAlpha = 1 - Math.exp(-forceTravel / sigmaForce);
    this.transientFx += (target.fx - this.transientFx) * forceAlpha;
    this.transientFy += (target.fy - this.transientFy) * forceAlpha;
    this.transientMz += (target.aligningTorque - this.transientMz) * forceAlpha;
'''
new_force = '''    const lateralForceSigma = Math.max(0.025, this.tireConfig.relaxationLength * 0.55);
    const longitudinalForceSigma = Math.max(
      0.018,
      this.tireConfig.longitudinalForceRelaxationLength ?? lateralForceSigma
    );
    const forceTravel = Math.max(2.5, Math.abs(longitudinalVelocity)) * dt;
    const lateralForceAlpha = 1 - Math.exp(-forceTravel / lateralForceSigma);
    const longitudinalForceAlpha = 1 - Math.exp(-forceTravel / longitudinalForceSigma);
    this.transientFx += (target.fx - this.transientFx) * longitudinalForceAlpha;
    this.transientFy += (target.fy - this.transientFy) * lateralForceAlpha;
    this.transientMz += (target.aligningTorque - this.transientMz) * lateralForceAlpha;
'''
if old_force in s:
    s = s.replace(old_force, new_force)
p.write_text(s)

# Vehicle: pass the optional M5 longitudinal transient calibration into every tire.
p = Path('src/physics/Vehicle.ts')
s = p.read_text()
needle = '      relaxationLength: this.config.relaxationLength,\n'
replacement = '''      relaxationLength: this.config.relaxationLength,
      longitudinalRelaxationLength: (this.config as any).longitudinalRelaxationLength,
      longitudinalForceRelaxationLength: (this.config as any).longitudinalForceRelaxationLength,
'''
# Constructor has front and rear tire configs; replace both occurrences if not present.
if 'longitudinalRelaxationLength: (this.config as any).longitudinalRelaxationLength' not in s:
    s = s.replace(needle, replacement)

set_needle = '        relaxationLength: newConfig.relaxationLength,\n'
set_replacement = '''        relaxationLength: newConfig.relaxationLength,
        longitudinalRelaxationLength: (newConfig as any).longitudinalRelaxationLength,
        longitudinalForceRelaxationLength: (newConfig as any).longitudinalForceRelaxationLength,
'''
if 'longitudinalRelaxationLength: (newConfig as any).longitudinalRelaxationLength' not in s:
    s = s.replace(set_needle, set_replacement)
p.write_text(s)

# M5 calibration: shorter longitudinal transients than lateral turn-in. These are
# explicit calibration estimates and are exercised by the launch diagnostic sweep.
p = Path('src/physics/m5G90.ts')
s = p.read_text()
if 'longitudinalRelaxationLength:' not in s:
    s = s.replace(
        '  relaxationLength: 0.19,\n',
        '  relaxationLength: 0.19,\n  longitudinalRelaxationLength: 0.065,\n  longitudinalForceRelaxationLength: 0.036,\n'
    )
p.write_text(s)
