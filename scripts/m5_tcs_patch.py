from pathlib import Path

# Make TCS wheel-slip thresholds and response rates calibration inputs. Defaults
# preserve existing vehicles; the M5 can use a faster near-actuator controller.
p = Path('src/physics/DriverAids.ts')
s = p.read_text()
if 'tcsSportSlipThreshold?: number;' not in s:
    s = s.replace(
        '  steerSpeedReduction: number;\n',
        '''  steerSpeedReduction: number;
  tcsSportSlipThreshold?: number;
  tcsFullSlipThreshold?: number;
  tcsSportResponse?: number;
  tcsFullResponse?: number;
  tcsSportGain?: number;
  tcsFullGain?: number;
'''
    )
s = s.replace(
    """    const isSport = this.config.tcsMode === 'SPORT';
    const tcsThreshold = isSport ? 0.19 : 0.12;
""",
    """    const isSport = this.config.tcsMode === 'SPORT';
    const tcsThreshold = isSport
      ? (this.config.tcsSportSlipThreshold ?? 0.19)
      : (this.config.tcsFullSlipThreshold ?? 0.12);
"""
)
s = s.replace(
    """      const gain = isSport ? 2.0 : 3.0;
      const maxReduction = isSport ? 0.72 : 0.88;
      const targetReduction = Math.min(maxReduction, excess * gain);
      const response = isSport ? 10.0 : 16.0;
""",
    """      const gain = isSport
        ? (this.config.tcsSportGain ?? 2.0)
        : (this.config.tcsFullGain ?? 3.0);
      const maxReduction = isSport ? 0.72 : 0.88;
      const targetReduction = Math.min(maxReduction, excess * gain);
      const response = isSport
        ? (this.config.tcsSportResponse ?? 10.0)
        : (this.config.tcsFullResponse ?? 16.0);
"""
)
p.write_text(s)

# Pass optional TCS calibration through Vehicle construction and live config updates.
p = Path('src/physics/Vehicle.ts')
s = p.read_text()
ctor_anchor = '      steerSpeedReduction: this.config.steerSpeedReduction,\n'
ctor_repl = '''      steerSpeedReduction: this.config.steerSpeedReduction,
      tcsSportSlipThreshold: (this.config as any).tcsSportSlipThreshold,
      tcsFullSlipThreshold: (this.config as any).tcsFullSlipThreshold,
      tcsSportResponse: (this.config as any).tcsSportResponse,
      tcsFullResponse: (this.config as any).tcsFullResponse,
      tcsSportGain: (this.config as any).tcsSportGain,
      tcsFullGain: (this.config as any).tcsFullGain,
'''
if 'tcsSportSlipThreshold: (this.config as any).tcsSportSlipThreshold' not in s:
    s = s.replace(ctor_anchor, ctor_repl)

set_anchor = '      steerSpeedReduction: newConfig.steerSpeedReduction,\n'
set_repl = '''      steerSpeedReduction: newConfig.steerSpeedReduction,
      tcsSportSlipThreshold: (newConfig as any).tcsSportSlipThreshold,
      tcsFullSlipThreshold: (newConfig as any).tcsFullSlipThreshold,
      tcsSportResponse: (newConfig as any).tcsSportResponse,
      tcsFullResponse: (newConfig as any).tcsFullResponse,
      tcsSportGain: (newConfig as any).tcsSportGain,
      tcsFullGain: (newConfig as any).tcsFullGain,
'''
if 'tcsSportSlipThreshold: (newConfig as any).tcsSportSlipThreshold' not in s:
    s = s.replace(set_anchor, set_repl)
p.write_text(s)
