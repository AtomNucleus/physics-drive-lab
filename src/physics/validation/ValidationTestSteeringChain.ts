import { writeLineChartSvg } from './ValidationArtifacts';
import {
  DT, NEUTRAL, RAD_TO_DEG, basicRow, makeSim, mean, setSpeed, writeTelemetry,
  type CorrectedValidationResult,
} from './CorrectedValidationCommon';

function withSteeringRow(sim: ReturnType<typeof makeSim>, t: number, controls: any) {
  const row = basicRow(sim, t, controls);
  const rack = sim.suspensionKinematics.steeringDynamics.telemetry;
  Object.assign(row, {
    steering_wheel_deg: rack.steeringWheelAngleRad * RAD_TO_DEG,
    steering_wheel_target_deg: rack.targetSteeringWheelAngleRad * RAD_TO_DEG,
    steering_wheel_rate_deg_s: rack.steeringWheelVelocityRadS * RAD_TO_DEG,
    rack_center_deg: rack.rackCenterAngleRad * RAD_TO_DEG,
    rack_rate_deg_s: rack.rackAngularVelocityRadS * RAD_TO_DEG,
    rack_position_m: rack.rackDisplacementM,
    rack_accel_deg_s2: rack.rackAngularAccelerationRadS2 * RAD_TO_DEG,
    steering_compliance_fl_deg: rack.leftComplianceRad * RAD_TO_DEG,
    steering_compliance_fr_deg: rack.rightComplianceRad * RAD_TO_DEG,
    steering_eps_torque_nm: rack.torques.epsAssistSteeringWheelNm,
    steering_driver_torque_nm: rack.torques.driverSteeringWheelNm,
    steering_road_sat_nm: rack.torques.tireSelfAligningRoadNm,
    steering_mechanical_trail_nm: rack.torques.casterMechanicalTrailRoadNm,
    steering_net_rack_torque_nm: rack.torques.netRackRoadNm,
  });
  return row;
}

function runManeuver(
  speedKmh: number,
  seconds: number,
  driver: (t: number) => number
): Record<string, unknown>[] {
  const sim = makeSim();
  setSpeed(sim, speedKmh / 3.6);
  const rows: Record<string, unknown>[] = [];
  for (let i = 0; i < Math.round(seconds / DT); i++) {
    const controls = { ...NEUTRAL, steer: driver(i * DT) };
    sim.stepExplicit(controls as any, 1);
    rows.push(withSteeringRow(sim, i * DT, controls));
  }
  return rows;
}

function tailMeanAbs(rows: Record<string, unknown>[], key: string, seconds = 0.30) {
  const count = Math.max(1, Math.round(seconds / DT));
  return mean(rows.slice(-count).map((row) => Math.abs(Number(row[key]))));
}

function meanRoadWheelAbs(rows: Record<string, unknown>[], seconds = 0.30) {
  const count = Math.max(1, Math.round(seconds / DT));
  return mean(rows.slice(-count).map((row) =>
    Math.abs((Number(row.fl_steer_deg) + Number(row.fr_steer_deg)) * 0.5)
  ));
}

function relativeMirrorError(a: number, b: number) {
  return Math.abs(Math.abs(a) - Math.abs(b)) / Math.max(1e-6, Math.max(Math.abs(a), Math.abs(b)));
}

export function runSteeringChainValidation(
  artifactDir: string,
  skidpad: CorrectedValidationResult,
  stepSteer: CorrectedValidationResult
): CorrectedValidationResult {
  const left = runManeuver(80, 1.25, () => 0.07);
  const right = runManeuver(80, 1.25, () => -0.07);

  const leftSteeringWheel = tailMeanAbs(left, 'steering_wheel_deg');
  const rightSteeringWheel = tailMeanAbs(right, 'steering_wheel_deg');
  const leftRoadWheel = meanRoadWheelAbs(left);
  const rightRoadWheel = meanRoadWheelAbs(right);
  const leftYaw = tailMeanAbs(left, 'yaw_rate_deg_s');
  const rightYaw = tailMeanAbs(right, 'yaw_rate_deg_s');
  const leftLatG = tailMeanAbs(left, 'lateral_g');
  const rightLatG = tailMeanAbs(right, 'lateral_g');

  const steeringWheelSymmetryError = relativeMirrorError(leftSteeringWheel, rightSteeringWheel);
  const roadWheelSymmetryError = relativeMirrorError(leftRoadWheel, rightRoadWheel);
  const yawSymmetryError = relativeMirrorError(leftYaw, rightYaw);
  const lateralGSymmetryError = relativeMirrorError(leftLatG, rightLatG);

  const slalom = runManeuver(80, 4.0, (t) => 0.075 * Math.sin(2 * Math.PI * 0.65 * t));
  const reversal = runManeuver(80, 3.0, (t) => {
    if (t < 0.55) return 0.08;
    if (t < 1.10) return -0.08;
    if (t < 1.65) return 0.08;
    if (t < 2.20) return -0.08;
    return 0;
  });

  const finite = [...left, ...right, ...slalom, ...reversal].every((row) =>
    Object.values(row).every((value) => typeof value !== 'number' || Number.isFinite(value))
  );
  const maxRackRateDegS = Math.max(...[...slalom, ...reversal].map((row) => Math.abs(Number(row.rack_rate_deg_s))));
  const maxComplianceDeg = Math.max(...[...slalom, ...reversal].flatMap((row) => [
    Math.abs(Number(row.steering_compliance_fl_deg)),
    Math.abs(Number(row.steering_compliance_fr_deg)),
  ]));
  const maxSlalomYawDegS = Math.max(...slalom.map((row) => Math.abs(Number(row.yaw_rate_deg_s))));
  const maxReversalYawDegS = Math.max(...reversal.map((row) => Math.abs(Number(row.yaw_rate_deg_s))));

  const symmetryCoherent =
    steeringWheelSymmetryError < 0.05 &&
    roadWheelSymmetryError < 0.06 &&
    yawSymmetryError < 0.10 &&
    lateralGSymmetryError < 0.10;
  const transientCoherent = finite && maxComplianceDeg < 0.40 && maxRackRateDegS < 400;
  const coherent = symmetryCoherent && transientCoherent;

  const telemetryFile = writeTelemetry(artifactDir, 'steering-chain-rapid-reversal', reversal);
  const graph = `${artifactDir}/steering-chain-rapid-reversal.svg`;
  writeLineChartSvg(graph, {
    title: '80 km/h rapid reversal — physical steering chain',
    subtitle: 'Driver target drives column/rack dynamics; no yaw-state term is used by the steering subsystem',
    xLabel: 'time (s)',
    yLabel: 'scaled response',
    x: reversal.map((row) => Number(row.time_s)),
    series: [
      { name: 'road-wheel deg', values: reversal.map((row) => (Number(row.fl_steer_deg) + Number(row.fr_steer_deg)) * 0.5) },
      { name: 'rack rate deg/s ÷ 10', values: reversal.map((row) => Number(row.rack_rate_deg_s) / 10) },
      { name: 'yaw deg/s', values: reversal.map((row) => Number(row.yaw_rate_deg_s)) },
    ],
  });

  const stepMetrics = stepSteer.metrics;
  const metrics: Record<string, number | string | null> = {
    steeringWheelAngleSlopeDegPerG: Number(skidpad.metrics.steeringWheelAngleSlopeDegPerG),
    roadWheelAngleSlopeDegPerG: Number(skidpad.metrics.roadWheelAngleSlopeDegPerG),
    roadWheelUndersteerGradientDegPerG: Number(skidpad.metrics.roadWheelUndersteerGradientDegPerG),
    leftRightSteeringWheelSymmetryErrorPercent: steeringWheelSymmetryError * 100,
    leftRightRoadWheelSymmetryErrorPercent: roadWheelSymmetryError * 100,
    leftRightYawSymmetryErrorPercent: yawSymmetryError * 100,
    leftRightLateralGSymmetryErrorPercent: lateralGSymmetryError * 100,
    slalomMaxRackRateDegS: Math.max(...slalom.map((row) => Math.abs(Number(row.rack_rate_deg_s)))),
    slalomPeakYawRateDegS: maxSlalomYawDegS,
    rapidReversalMaxRackRateDegS: Math.max(...reversal.map((row) => Math.abs(Number(row.rack_rate_deg_s)))),
    rapidReversalPeakYawRateDegS: maxReversalYawDegS,
    maxSteeringComplianceDeg: maxComplianceDeg,
  };

  for (const speed of [30, 50, 80, 100]) {
    for (const metric of [
      'commandToRoadWheel10Sec',
      'commandToYaw10Sec',
      'yawRiseTimeSec',
      'yawRateGainDegSPerRoadWheelDeg',
      'yawRateGainDegSPerSteeringWheelDeg',
      'roadWheelAngleDegPerG',
      'steeringWheelAngleDegPerG',
    ]) {
      const key = `${speed}Kmh_${metric}`;
      const value = stepMetrics[key];
      metrics[key] = typeof value === 'number' && Number.isFinite(value) ? value : null;
    }
  }

  return {
    id: 'steering-chain',
    name: 'Physical steering rack/column chain validation',
    status: coherent ? 'NO REFERENCE DATA' : 'FAIL',
    validationClass: 'internal-regression',
    blocking: !coherent,
    summary: coherent
      ? `Physical steering chain remained stable through step steer, slalom, rapid reversal and mirrored 80 km/h turns; external G90 dynamic targets are unavailable.`
      : 'Physical steering-chain invariant failed; inspect rack dynamics/symmetry before tuning or merge.',
    metrics,
    diagnostics: [
      ...(!symmetryCoherent ? [
        `Left/right symmetry exceeded invariant tolerances: steering wheel ${(steeringWheelSymmetryError * 100).toFixed(1)}%, road wheel ${(roadWheelSymmetryError * 100).toFixed(1)}%, yaw ${(yawSymmetryError * 100).toFixed(1)}%, lateral g ${(lateralGSymmetryError * 100).toFixed(1)}%.`,
      ] : []),
      ...(!transientCoherent ? [
        'Slalom/rapid-reversal rack state became non-finite, exceeded the internal rate guardrail, or steering compliance exceeded 0.4°.',
      ] : []),
      'No artificial yaw assistance is part of the steering equations; rack torque depends on driver/column torque, EPS, tire aligning torque, steering-axis force moment, damping/friction and end stops.',
      'NO REFERENCE DATA: no trustworthy G90 steering-wheel-angle-vs-g, road-wheel-angle-vs-g, understeer-gradient, step-delay, yaw-rate-gain, slalom or rapid-reversal target set is stored. These values are measured and reported, not scored against invented BMW numbers.',
    ],
    telemetryFile,
    graphFiles: [graph],
  };
}
