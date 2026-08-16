from pathlib import Path

p = Path('src/physics/Vehicle.ts')
s = p.read_text()

# Powertrain constructor: preserve defaults for other cars, but pass optional
# automatic shift calibration fields when a vehicle preset provides them.
ctor_anchor = '''      automaticTorqueConverter: (this.config as any).automaticTorqueConverter,
      autoBlipDownshift: this.config.autoBlipDownshift,
'''
ctor_repl = '''      automaticTorqueConverter: (this.config as any).automaticTorqueConverter,
      shiftDurationSec: (this.config as any).shiftDurationSec,
      shiftTorqueMultiplier: (this.config as any).shiftTorqueMultiplier,
      autoBlipDownshift: this.config.autoBlipDownshift,
'''
if 'shiftDurationSec: (this.config as any).shiftDurationSec' not in s:
    s = s.replace(ctor_anchor, ctor_repl)

# Powertrain live config updates.
set_anchor = '''      automaticTorqueConverter: (newConfig as any).automaticTorqueConverter,
      autoBlipDownshift: newConfig.autoBlipDownshift,
'''
set_repl = '''      automaticTorqueConverter: (newConfig as any).automaticTorqueConverter,
      shiftDurationSec: (newConfig as any).shiftDurationSec,
      shiftTorqueMultiplier: (newConfig as any).shiftTorqueMultiplier,
      autoBlipDownshift: newConfig.autoBlipDownshift,
'''
if 'shiftDurationSec: (newConfig as any).shiftDurationSec' not in s:
    s = s.replace(set_anchor, set_repl)

# Driver-aid constructor: optional near-actuator TCS tuning. Existing vehicles
# continue to use DriverAids defaults when these values are undefined.
aids_anchor = '''      steerSpeed: this.config.steerSpeed,
      steerSpeedReduction: this.config.steerSpeedReduction,
    });
'''
aids_repl = '''      steerSpeed: this.config.steerSpeed,
      steerSpeedReduction: this.config.steerSpeedReduction,
      tcsSportSlipThreshold: (this.config as any).tcsSportSlipThreshold,
      tcsFullSlipThreshold: (this.config as any).tcsFullSlipThreshold,
      tcsSportResponse: (this.config as any).tcsSportResponse,
      tcsFullResponse: (this.config as any).tcsFullResponse,
      tcsSportGain: (this.config as any).tcsSportGain,
      tcsFullGain: (this.config as any).tcsFullGain,
    });
'''
if 'tcsSportSlipThreshold: (this.config as any).tcsSportSlipThreshold' not in s:
    s = s.replace(aids_anchor, aids_repl, 1)

# Driver-aid live config updates.
aids_set_anchor = '''      steerSpeed: newConfig.steerSpeed,
      steerSpeedReduction: newConfig.steerSpeedReduction,
    };
'''
aids_set_repl = '''      steerSpeed: newConfig.steerSpeed,
      steerSpeedReduction: newConfig.steerSpeedReduction,
      tcsSportSlipThreshold: (newConfig as any).tcsSportSlipThreshold,
      tcsFullSlipThreshold: (newConfig as any).tcsFullSlipThreshold,
      tcsSportResponse: (newConfig as any).tcsSportResponse,
      tcsFullResponse: (newConfig as any).tcsFullResponse,
      tcsSportGain: (newConfig as any).tcsSportGain,
      tcsFullGain: (newConfig as any).tcsFullGain,
    };
'''
if 'tcsSportSlipThreshold: (newConfig as any).tcsSportSlipThreshold' not in s:
    s = s.replace(aids_set_anchor, aids_set_repl, 1)

p.write_text(s)
