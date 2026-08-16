from pathlib import Path

m5 = Path('src/physics/m5G90.ts')
if not m5.exists():
    m5.write_text("""import type { VehicleConfig } from '../types';

/** 2025 BMW M5 (G90) instrumented-test calibration. */
export const BMW_M5_2025_OVERRIDES: Partial<VehicleConfig> & Record<string, any> = {
  mass: 2381.8135,
  weightDistributionFront: 0.545,
  centerOfGravityHeight: 0.52,
  wheelbase: 3.00482,
  trackWidth: 1.67259,
  wheelRadius: 0.369,
  wheelInertia: 2.10,
  unsprungMassCorner: 55,

  suspensionRestLength: 0.34,
  suspensionStiffness: 62000,
  suspensionDamping: 5000,
  suspensionDampingLowSpeed: 5200,
  suspensionDampingHighSpeed: 3000,
  suspensionReboundDamping: 6500,
  bumpStopStiffness: 70000,
  bumpStopTravelThreshold: 0.80,
  rollStiffnessFront: 46000,
  rollStiffnessRear: 40000,
  antiRollCrossCoupling: 0.30,
  camberStaticFront: -1.5,
  camberStaticRear: -1.2,
  camberGain: 7.5,
  antiDiveFront: 0.45,
  antiSquatRear: 0.35,
  bodyRollMultiplier: 1.15,
  bodyPitchMultiplier: 1.05,

  tireGripFront: 1.21,
  tireGripRear: 1.20,
  tireStiffness: 15.0,
  tireLoadSensitivity: 0.000030,
  slideFrictionMultiplier: 0.83,
  relaxationLength: 0.19,
  tirePneumaticTrailMax: 0.030,
  tireSidewallStiffness: 230000,
  tireVerticalStiffness: 280000,
  tireVerticalDamping: 1800,
  tireBasePressure: 35.0,
  optimalTireTemp: 75,
  driftAssist: 0.0,

  aeroDownforceFront: 120,
  aeroDownforceRear: 180,
  aeroDragCoeff: 0.35,
  aeroCopPitchSensitivity: 0.04,
  groundEffectUnderbody: false,
  groundEffectMaxDownforce: 0,
  drsEnabled: false,
  airbrakeEnabled: false,

  drivetrain: 'AWD',
  differentialType: 'TORQUE_VECTOR',
  centerFrontTorqueRatio: 0.32,
  diffPowerRamp: 0.88,
  diffCoastRamp: 0.48,
  diffPreloadTorque: 100,
  diffLockRatio: 0.88,

  maxTorque: 700,
  maxRpm: 7200,
  idleRpm: 750,
  revLimiterRpm: 7100,
  flywheelInertia: 0.28,
  engineBrakingTorque: 100,
  clutchBiteRate: 20.0,
  maxClutchTorque: 1500,
  transmissionEfficiency: 0.94,
  turboBoostMaxPsi: 18.0,
  turboSpoolRate: 12.0,
  wastegatePressurePsi: 17.5,
  reverseRatio: -3.97,
  forwardGearRatios: [5.00, 3.20, 2.14, 1.72, 1.30, 1.00, 0.83, 0.64],
  gearRatios: [-3.97, 5.00, 3.20, 2.14, 1.72, 1.30, 1.00, 0.83, 0.64],
  finalDriveRatio: 3.31,

  brakeForce: 18200,
  handbrakeForce: 10000,
  brakeBiasFront: 0.62,
  ackermannRatio: 0.90,
  maxSteerAngle: 0.58,
  steerSpeed: 4.8,
  steerSpeedReduction: 0.60,
  rearSteerMaxDeg: 1.5,
  rearSteerTransitionSpeedMs: 20.0,
  absMode: 'SPORT',
  tcsMode: 'SPORT',
  launchControlEnabled: true,
};

export const BMW_M5_2025_TARGETS = {
  zeroTo60MphSec: 3.0,
  zeroTo100MphSec: 6.7,
  quarterMileSec: 10.9,
  quarterMileTrapMph: 130,
  braking70To0Ft: 157,
  braking100To0Ft: 324,
  skidpadG: 0.98,
};
""")

p = Path('src/physics/Vehicle.ts')
s = p.read_text()
s = s.replace('    const tireRadius = 0.33;\n    const wheelInertia = 1.25;', '    const tireRadius = this.config.wheelRadius;\n    const wheelInertia = this.config.wheelInertia;')
s = s.replace('      gearRatios: this.config.gearRatios,\n      finalDriveRatio: this.config.finalDriveRatio,', '      reverseRatio: this.config.reverseRatio,\n      forwardGearRatios: this.config.forwardGearRatios,\n      gearRatios: this.config.gearRatios,\n      finalDriveRatio: this.config.finalDriveRatio,\n      maxClutchTorque: this.config.maxClutchTorque,\n      transmissionEfficiency: this.config.transmissionEfficiency,')
s = s.replace('      drivetrain: this.config.drivetrain,\n    });', '      drivetrain: this.config.drivetrain,\n      frontTorqueRatio: (this.config as any).centerFrontTorqueRatio,\n    });', 1)
s = s.replace('      gearRatios: newConfig.gearRatios,\n      finalDriveRatio: newConfig.finalDriveRatio,', '      reverseRatio: newConfig.reverseRatio,\n      forwardGearRatios: newConfig.forwardGearRatios,\n      gearRatios: newConfig.gearRatios,\n      finalDriveRatio: newConfig.finalDriveRatio,\n      maxClutchTorque: newConfig.maxClutchTorque,\n      transmissionEfficiency: newConfig.transmissionEfficiency,')
s = s.replace('      drivetrain: newConfig.drivetrain,\n    };', '      drivetrain: newConfig.drivetrain,\n      frontTorqueRatio: (newConfig as any).centerFrontTorqueRatio,\n    };', 1)
s = s.replace('      this.wheels[i].tireConfig = {', '      (this.wheels[i] as any).radius = newConfig.wheelRadius;\n      (this.wheels[i] as any).inertia = newConfig.wheelInertia;\n      this.wheels[i].tireConfig = {')
s = s.replace('    this.wheels[2].steerAngle = 0;\n    this.wheels[3].steerAngle = 0;', """    const meanFrontSteer = (steerOut.steerFL + steerOut.steerFR) * 0.5;
    const rearMax = (((this.config as any).rearSteerMaxDeg ?? 0) * Math.PI) / 180;
    const rearTransition = Math.max(1, (this.config as any).rearSteerTransitionSpeedMs ?? 20);
    const speedAbs = Math.abs(forwardSpeed);
    const phase = PhysicsMath.clamp((speedAbs - (rearTransition - 5)) / 10, 0, 1);
    const lowSpeedRear = -Math.sign(meanFrontSteer) * Math.min(Math.abs(meanFrontSteer) * 0.35, rearMax);
    const highSpeedRear = Math.sign(meanFrontSteer) * Math.min(Math.abs(meanFrontSteer) * 0.18, rearMax);
    const rearSteer = lowSpeedRear + (highSpeedRear - lowSpeedRear) * phase;
    this.wheels[2].steerAngle = rearSteer;
    this.wheels[3].steerAngle = rearSteer;""")
s = s.replace('      0.33,\n      this.config.tireVerticalStiffness,', '      this.config.wheelRadius,\n      this.config.tireVerticalStiffness,')
p.write_text(s)

p = Path('src/physics/Differential.ts')
s = p.read_text()
s = s.replace("  drivetrain: DrivetrainType; // 'RWD' | 'FWD' | 'AWD'\n}", "  drivetrain: DrivetrainType; // 'RWD' | 'FWD' | 'AWD'\n  frontTorqueRatio?: number;\n}")
s = s.replace("    // AWD: Center Differential with 40/60 Front/Rear torque split\n    const frontRatio = 0.40;\n    const rearRatio = 0.60;", """    // Adaptive center coupling: rear-biased baseline, then send more torque
    // forward when rear-axle overspeed indicates that the rear tires need help.
    const baseFrontRatio = PhysicsMath.clamp(this.config.frontTorqueRatio ?? 0.40, 0.20, 0.50);
    const frontOmega = (Math.abs(omegaFL) + Math.abs(omegaFR)) * 0.5;
    const rearOmega = (Math.abs(omegaRL) + Math.abs(omegaRR)) * 0.5;
    const axleSpeedRef = Math.max(1.0, (frontOmega + rearOmega) * 0.5);
    const rearOverspeed = (rearOmega - frontOmega) / axleSpeedRef;
    const frontRatio = PhysicsMath.clamp(baseFrontRatio + rearOverspeed * 0.28, 0.20, 0.50);
    const rearRatio = 1.0 - frontRatio;""")
p.write_text(s)

p = Path('src/physics/vehiclePresets.ts')
s = p.read_text()
if "./m5G90" not in s:
    s = s.replace("import { VehicleConfig, VehiclePreset } from '../types';", "import { VehicleConfig, VehiclePreset } from '../types';\nimport { BMW_M5_2025_OVERRIDES } from './m5G90';")
marker = "export const VEHICLE_PRESETS: Record<string, VehiclePreset> = {\n"
if 'm5G90:' not in s:
    entry = """  m5G90: {
    name: '2025 BMW M5 (G90)',
    tagline: '5,251 lb measured G90 • 717 hp M Hybrid • rear-biased M xDrive',
    description: 'Calibrated from instrumented 2025 G90 M5 measurements, BMW gearing and tire geometry, adaptive rear-biased AWD, and active rear steering.',
    color: '#111827',
    config: BMW_M5_2025_OVERRIDES,
  },
"""
    s = s.replace(marker, marker + entry)
p.write_text(s)

p = Path('src/App.tsx')
s = p.read_text()
if "./physics/m5G90" not in s:
    s = s.replace("import { DEFAULT_VEHICLE_CONFIG, VEHICLE_PRESETS } from './physics/vehiclePresets';", "import { DEFAULT_VEHICLE_CONFIG, VEHICLE_PRESETS } from './physics/vehiclePresets';\nimport { BMW_M5_2025_OVERRIDES } from './physics/m5G90';")
if 'const INITIAL_PRESET_KEY' not in s:
    s = s.replace('export default function App() {', "const INITIAL_PRESET_KEY = 'm5G90';\nconst INITIAL_CONFIG: VehicleConfig = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES } as VehicleConfig;\n\nexport default function App() {")
s = s.replace('useState<VehicleConfig>(DEFAULT_VEHICLE_CONFIG)', 'useState<VehicleConfig>(INITIAL_CONFIG)')
s = s.replace("useState<string>('sportGT')", 'useState<string>(INITIAL_PRESET_KEY)')
s = s.replace("useState<string>('#2563eb')", "useState<string>('#111827')")
s = s.replace('new VehiclePhysicsEngine(DEFAULT_VEHICLE_CONFIG)', 'new VehiclePhysicsEngine(INITIAL_CONFIG)')
p.write_text(s)
