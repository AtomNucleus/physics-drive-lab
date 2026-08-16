from pathlib import Path

# Lock the measured/balanced G90 calibration selected from the read-only sweeps.
p = Path('src/physics/m5G90.ts')
s = p.read_text()
replacements = {
    '  longitudinalRelaxationLength: 0.08,': '  longitudinalRelaxationLength: 0.12,',
    '  longitudinalForceRelaxationLength: 0.045,': '  longitudinalForceRelaxationLength: 0.066,',
    '  aeroDownforceFront: 120,': '  aeroDownforceFront: 0,',
    '  aeroDownforceRear: 180,': '  aeroDownforceRear: 0,',
    '  centerFrontTorqueRatio: 0.32,': '  centerFrontTorqueRatio: 0.40,',
    '  brakeForce: 18200,': '  brakeForce: 10500,',
    '  drivelineInputInertia: 0.55,': '  drivelineInputInertia: 0.35,',
}
for old, new in replacements.items():
    s = s.replace(old, new)

if '  shiftDurationSec:' not in s:
    s = s.replace(
        '  automaticTorqueConverter: true,\n',
        '  automaticTorqueConverter: true,\n  shiftDurationSec: 0.07,\n  shiftTorqueMultiplier: 0.80,\n'
    )
if '  tcsSportSlipThreshold:' not in s:
    s = s.replace(
        "  tcsMode: 'SPORT',\n",
        "  tcsMode: 'SPORT',\n  tcsSportSlipThreshold: 0.16,\n  tcsSportResponse: 30.0,\n  tcsSportGain: 2.6,\n"
    )
for unused in [
    '  launchSlipTarget: 0.12,\n',
    '  launchTractionUtilization: 0.98,\n',
    '  launchControlEndSpeedMs: 30.0,\n',
]:
    s = s.replace(unused, '')
p.write_text(s)

# Wire M5-specific shift and TCS calibration through normal runtime config.
p = Path('src/physics/Vehicle.ts')
s = p.read_text()

constructor_powertrain = """      automaticTorqueConverter: (this.config as any).automaticTorqueConverter,
      autoBlipDownshift: this.config.autoBlipDownshift,
"""
if 'shiftDurationSec: (this.config as any).shiftDurationSec' not in s:
    s = s.replace(
        constructor_powertrain,
        """      automaticTorqueConverter: (this.config as any).automaticTorqueConverter,
      shiftDurationSec: (this.config as any).shiftDurationSec,
      shiftTorqueMultiplier: (this.config as any).shiftTorqueMultiplier,
      autoBlipDownshift: this.config.autoBlipDownshift,
"""
    )

set_powertrain = """      automaticTorqueConverter: (newConfig as any).automaticTorqueConverter,
      autoBlipDownshift: newConfig.autoBlipDownshift,
"""
if 'shiftDurationSec: (newConfig as any).shiftDurationSec' not in s:
    s = s.replace(
        set_powertrain,
        """      automaticTorqueConverter: (newConfig as any).automaticTorqueConverter,
      shiftDurationSec: (newConfig as any).shiftDurationSec,
      shiftTorqueMultiplier: (newConfig as any).shiftTorqueMultiplier,
      autoBlipDownshift: newConfig.autoBlipDownshift,
"""
    )

constructor_aids = """      steerSpeed: this.config.steerSpeed,
      steerSpeedReduction: this.config.steerSpeedReduction,
    });
"""
if 'tcsSportSlipThreshold: (this.config as any).tcsSportSlipThreshold' not in s:
    s = s.replace(
        constructor_aids,
        """      steerSpeed: this.config.steerSpeed,
      steerSpeedReduction: this.config.steerSpeedReduction,
      tcsSportSlipThreshold: (this.config as any).tcsSportSlipThreshold,
      tcsFullSlipThreshold: (this.config as any).tcsFullSlipThreshold,
      tcsSportResponse: (this.config as any).tcsSportResponse,
      tcsFullResponse: (this.config as any).tcsFullResponse,
      tcsSportGain: (this.config as any).tcsSportGain,
      tcsFullGain: (this.config as any).tcsFullGain,
    });
""",
        1,
    )

set_aids = """      steerSpeed: newConfig.steerSpeed,
      steerSpeedReduction: newConfig.steerSpeedReduction,
    };
"""
if 'tcsSportSlipThreshold: (newConfig as any).tcsSportSlipThreshold' not in s:
    s = s.replace(
        set_aids,
        """      steerSpeed: newConfig.steerSpeed,
      steerSpeedReduction: newConfig.steerSpeedReduction,
      tcsSportSlipThreshold: (newConfig as any).tcsSportSlipThreshold,
      tcsFullSlipThreshold: (newConfig as any).tcsFullSlipThreshold,
      tcsSportResponse: (newConfig as any).tcsSportResponse,
      tcsFullResponse: (newConfig as any).tcsFullResponse,
      tcsSportGain: (newConfig as any).tcsSportGain,
      tcsFullGain: (newConfig as any).tcsFullGain,
    };
""",
        1,
    )

# Powertrain keeps derived gear arrays as runtime fields; update them when presets change.
gear_anchor = """    this.differential.config = {
"""
if 'this.powertrain.forwardGearRatios = [...newConfig.forwardGearRatios];' not in s:
    s = s.replace(
        gear_anchor,
        """    this.powertrain.forwardGearRatios = [...newConfig.forwardGearRatios];
    this.powertrain.reverseRatio = newConfig.reverseRatio;
    this.powertrain.finalDriveRatio = newConfig.finalDriveRatio;

    this.differential.config = {
""",
        1,
    )

p.write_text(s)
