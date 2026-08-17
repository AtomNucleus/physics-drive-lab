import type { VehicleConfig } from '../types';

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
  frontCasterDeg: 7.2,
  frontKingpinInclinationDeg: 7.0,
  antiDiveFront: 0.45,
  antiSquatRear: 0.35,
  // Render exactly the rigid-body pitch/roll produced by the suspension and tire forces.
  // No visual multiplier is allowed on the M5 calibration.
  bodyRollMultiplier: 1.0,
  bodyPitchMultiplier: 1.0,

  tireGripFront: 1.21,
  tireGripRear: 1.20,
  tireStiffness: 15.0,
  tireLoadSensitivity: 0.000030,
  slideFrictionMultiplier: 0.83,
  // The G90's heavy chassis should not see peak lateral tire force in the same
  // 120 Hz frame as a steering step. A longer lateral relaxation length gives the
  // contact patch/carcass time to take a set before load reaches the sprung body.
  // Longitudinal relaxation remains independent below so acceleration/braking
  // response is unchanged.
  relaxationLength: 0.50,
  longitudinalRelaxationLength: 0.12,
  longitudinalForceRelaxationLength: 0.066,
  tirePneumaticTrailMax: 0.030,
  tireSidewallStiffness: 230000,
  tireVerticalStiffness: 280000,
  tireVerticalDamping: 1800,
  tireBasePressure: 35.0,
  optimalTireTemp: 75,
  driftAssist: 0.0,

  // The test data does not provide a measured road-car downforce map, so do not
  // invent aerodynamic load merely to force a benchmark result.
  aeroDownforceFront: 0,
  aeroDownforceRear: 0,
  aeroDragCoeff: 0.35,
  aeroCopPitchSensitivity: 0.04,
  groundEffectUnderbody: false,
  groundEffectMaxDownforce: 0,
  drsEnabled: false,
  airbrakeEnabled: false,

  drivetrain: 'AWD',
  differentialType: 'TORQUE_VECTOR',
  centerFrontTorqueRatio: 0.40,
  diffPowerRamp: 0.88,
  diffCoastRamp: 0.48,
  diffPreloadTorque: 100,
  diffLockRatio: 0.88,

  // maxTorque is the pre-boost base curve in this engine. Turbo boost and the
  // low-rpm hybrid fill below produce the system delivery without inflating the
  // real car's ~130 mph quarter-mile trap speed.
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

  // Retain one speed-independent hydraulic calibration rather than adding a
  // low-speed brake multiplier solely to erase the remaining 70-0 test residual.
  brakeForce: 10800,
  handbrakeForce: 10000,
  brakeBiasFront: 0.60,

  // Steering geometry and ratio anchors. BMW publishes a 14.2:1 overall steering
  // ratio, 118.3 in wheelbase, 66.3 in front track, speed-sensitive M Servotronic
  // assistance and variable steering ratio. The single legacy trackWidth above is
  // the mean front/rear track used by chassis load-transfer code; steering uses the
  // actual published 66.3 in front track independently.
  ackermannRatio: 0.90,
  steeringFrontTrackM: 1.68402,
  maxSteerAngle: 0.58,
  steeringRatioCenter: 14.2,
  // BMW does not publish the full ratio curve. 12.6:1 at high rack travel is a
  // conservative variable-ratio calibration that retains ~14.2:1 around center.
  steeringRatioAtLock: 12.6,

  // The remaining rack/EPS/compliance values are explicit engineering calibration,
  // not claimed BMW measurements. The equivalent rack inertia includes reflected
  // column/steering-wheel inertia through the steering ratio.
  steeringRackHalfTravelM: 0.070,
  steeringRackEquivalentInertiaKgm2: 5.8,
  steeringRackDampingNmsPerRad: 46,
  steeringRackFrictionNm: 4.5,
  steeringRackMaxAngularSpeedRadS: 3.2,
  steeringRackMaxAngularAccelRadS2: 180,
  steeringColumnTorsionStiffnessNmPerRad: 4.0,
  steeringColumnTorsionDampingNmsPerRad: 0.18,
  steeringDriverMaxTorqueNm: 8.0,

  // EPS acts on measured driver/column torque; it does not overwrite tire/caster
  // feedback. Assistance fades with speed while the physical rack stops stay fixed.
  steeringEpsParkingGain: 20.0,
  steeringEpsHighSpeedGain: 9.0,
  steeringEpsFadeSpeedMs: 27.8,
  steeringEpsMaxAssistTorqueNm: 65,

  steeringStopStartFraction: 0.92,
  steeringStopStiffnessNmPerRad: 9000,
  steeringStopDampingNmsPerRad: 190,

  // Four elastic elements in series at each front corner. Their aggregate motion is
  // limited to <0.4 deg so contact-patch load can move effective toe subtly without
  // making the wheel visibly floppy.
  steeringTieRodStiffnessNmPerRad: 180000,
  steeringRackMountStiffnessNmPerRad: 250000,
  steeringControlArmBushingStiffnessNmPerRad: 140000,
  steeringTireCarcassStiffnessNmPerRad: 110000,
  steeringComplianceDampingNmsPerRad: 1800,
  steeringMaxComplianceRad: 0.0065,

  // Legacy DriverAids values remain for direct Vehicle-only compatibility tests;
  // Simulation runtime steering is replaced by PhysicalSteeringSystem.
  steerSpeed: 4.8,
  steerSpeedReduction: 0.60,
  rearSteerMaxDeg: 1.5,
  rearSteerTransitionSpeedMs: 20.0,
  absMode: 'FULL',
  tcsMode: 'SPORT',
  tcsSportSlipThreshold: 0.16,
  tcsSportResponse: 30.0,
  tcsSportGain: 2.6,

  launchControlEnabled: true,
  launchControlRpm: 3000,
  lowSpeedTorqueFillNm: 600,
  torqueFillFadeRpm: 3200,
  automaticTorqueConverter: true,
  shiftDurationSec: 0.07,
  shiftTorqueMultiplier: 0.80,
  drivelineInputInertia: 0.35,
  drivelineInertiaCoupling: 1.0,
};

/** Published Car and Driver acceleration figures; those figures exclude 1-foot rollout. */
export const BMW_M5_2025_TARGETS = {
  zeroTo30MphSec: 1.1,
  zeroTo60MphSec: 3.0,
  zeroTo100MphSec: 6.7,
  quarterMileSec: 10.9,
  quarterMileTrapMph: 130,
  braking70To0Ft: 157,
  braking100To0Ft: 324,
  skidpadG: 0.98,
};

/** Equivalent true-standing-start targets for the simulator's zero-speed stopwatch. */
export const BMW_M5_2025_STANDING_TARGETS = {
  zeroTo30MphSec: 1.3,
  zeroTo60MphSec: 3.2,
  zeroTo100MphSec: 6.9,
  quarterMileSec: 11.1,
  quarterMileTrapMph: 130,
};
