import { writeLineChartSvg } from './ValidationArtifacts';
import {
  CONFIG, DT, DEG_TO_RAD, NEUTRAL, RAD_TO_DEG, basicRow, makeSim, maxAbs,
  mean, setSpeed, writeTelemetry, type CorrectedValidationResult,
} from './CorrectedValidationCommon';

function firstAtOrAbove(values: number[], threshold: number, times: number[]) {
  const index = values.findIndex((value) => value >= threshold);
  return index >= 0 ? times[index] : null;
}

function runStep(speedKmh: number) {
  const sim = makeSim();
  setSpeed(sim, speedKmh / 3.6);
  const rows: Record<string, unknown>[] = [];
  const steer = (3 * DEG_TO_RAD) / CONFIG.maxSteerAngle;

  for (let i = 0; i < Math.round(3 / DT); i++) {
    const controls = { ...NEUTRAL, steer: i * DT < 1.5 ? steer : 0 };
    sim.stepExplicit(controls as any, 1);
    // This row describes the post-step state, so timestamp it at the end of the
    // fixed step. Labeling the first post-input sample t=0 made finite steering
    // onset appear instantaneous even though the rack had already advanced 8.33 ms.
    const row = basicRow(sim, (i + 1) * DT, controls);
    const rack = sim.suspensionKinematics.steeringDynamics.telemetry;
    Object.assign(row, {
      steering_wheel_deg: rack.steeringWheelAngleRad * RAD_TO_DEG,
      steering_wheel_target_deg: rack.targetSteeringWheelAngleRad * RAD_TO_DEG,
      steering_wheel_rate_deg_s: rack.steeringWheelVelocityRadS * RAD_TO_DEG,
      rack_center_deg: rack.rackCenterAngleRad * RAD_TO_DEG,
      rack_rate_deg_s: rack.rackAngularVelocityRadS * RAD_TO_DEG,
      rack_accel_deg_s2: rack.rackAngularAccelerationRadS2 * RAD_TO_DEG,
      rack_position_m: rack.rackDisplacementM,
      steering_compliance_fl_deg: rack.leftComplianceRad * RAD_TO_DEG,
      steering_compliance_fr_deg: rack.rightComplianceRad * RAD_TO_DEG,
      steering_driver_torque_nm: rack.torques.driverSteeringWheelNm,
      steering_eps_torque_nm: rack.torques.epsAssistSteeringWheelNm,
      steering_road_sat_nm: rack.torques.tireSelfAligningRoadNm,
      steering_mechanical_trail_nm: rack.torques.casterMechanicalTrailRoadNm,
      steering_net_rack_torque_nm: rack.torques.netRackRoadNm,
    });
    rows.push(row);
  }

  const hold = rows.filter((row) => Number(row.time_s) >= 0.8 && Number(row.time_s) < 1.5);
  const steadyYaw = mean(hold.map((row) => Math.abs(Number(row.yaw_rate_deg_s))));
  const steadySteer = mean(hold.map((row) =>
    Math.abs((Number(row.fl_steer_deg) + Number(row.fr_steer_deg)) * 0.5)
  ));
  const steadySteeringWheel = mean(hold.map((row) => Math.abs(Number(row.steering_wheel_deg))));
  const steadyLateralG = mean(hold.map((row) => Math.abs(Number(row.lateral_g))));
  const steadySlip = mean(hold.map((row) =>
    0.5 * (Math.abs(Number(row.fl_slip_angle_deg)) + Math.abs(Number(row.fr_slip_angle_deg)))
  ));

  const times = rows.map((row) => Number(row.time_s));
  const steeringWheelSeries = rows.map((row) => Math.abs(Number(row.steering_wheel_deg)));
  const steerSeries = rows.map((row) =>
    Math.abs((Number(row.fl_steer_deg) + Number(row.fr_steer_deg)) * 0.5)
  );
  const slipSeries = rows.map((row) =>
    0.5 * (Math.abs(Number(row.fl_slip_angle_deg)) + Math.abs(Number(row.fr_slip_angle_deg)))
  );
  const yawSeries = rows.map((row) => Math.abs(Number(row.yaw_rate_deg_s)));

  const steeringWheel10 = firstAtOrAbove(steeringWheelSeries, steadySteeringWheel * 0.10, times);
  const steer10 = firstAtOrAbove(steerSeries, steadySteer * 0.10, times);
  const slip10 = firstAtOrAbove(slipSeries, steadySlip * 0.10, times);
  const yaw10 = firstAtOrAbove(yawSeries, steadyYaw * 0.10, times);
  const yaw90 = firstAtOrAbove(yawSeries, steadyYaw * 0.90, times);
  const peakYaw = Math.max(...rows
    .filter((row) => Number(row.time_s) < 1.5)
    .map((row) => Math.abs(Number(row.yaw_rate_deg_s))));
  const overshoot = steadyYaw > 1e-6 ? (peakYaw / steadyYaw - 1) * 100 : null;

  const releaseRows = rows.filter((row) => Number(row.time_s) >= 1.5);
  const settlingThreshold = steadyYaw * 0.05;
  const window = Math.round(0.2 / DT);
  let settlingTimeSec: number | null = null;
  for (let i = 0; i <= releaseRows.length - window; i++) {
    if (releaseRows.slice(i, i + window).every((row) => Math.abs(Number(row.yaw_rate_deg_s)) <= settlingThreshold)) {
      settlingTimeSec = Number(releaseRows[i].time_s) - 1.5;
      break;
    }
  }

  const yawRateGainRoadWheel = steadySteer > 1e-6 ? steadyYaw / steadySteer : null;
  const yawRateGainSteeringWheel = steadySteeringWheel > 1e-6 ? steadyYaw / steadySteeringWheel : null;
  const roadWheelAnglePerG = steadyLateralG > 1e-6 ? steadySteer / steadyLateralG : null;
  const steeringWheelAnglePerG = steadyLateralG > 1e-6 ? steadySteeringWheel / steadyLateralG : null;
  const commandToRoadWheel10Sec = steer10;
  const commandToYaw10Sec = yaw10;
  const chronologyCorrect = steeringWheel10 !== null && steer10 !== null && slip10 !== null && yaw10 !== null &&
    steeringWheel10 <= steer10 + DT && steer10 <= slip10 + DT && yaw10 >= slip10 + 0.5 * DT;

  return {
    speedKmh,
    rows,
    steadyYaw,
    steadySteer,
    steadySteeringWheel,
    steadyLateralG,
    steadySlip,
    steeringWheel10,
    steer10,
    slip10,
    yaw10,
    yaw90,
    commandToRoadWheel10Sec,
    commandToYaw10Sec,
    riseTimeSec: yaw10 !== null && yaw90 !== null ? yaw90 - yaw10 : null,
    overshoot,
    settlingTimeSec,
    yawRateGainRoadWheel,
    yawRateGainSteeringWheel,
    roadWheelAnglePerG,
    steeringWheelAnglePerG,
    chronologyCorrect,
  };
}

export function runStepSteerValidation(artifactDir: string): CorrectedValidationResult {
  const runs = [30, 50, 80, 100].map(runStep);
  const run80 = runs.find((run) => run.speedKmh === 80)!;
  const coherent = runs.every((run) =>
    run.chronologyCorrect && (run.overshoot ?? 999) < 120 &&
    maxAbs(run.rows.map((row) => Number(row.roll_deg))) < 12
  );

  const telemetryFile = writeTelemetry(artifactDir, 'step-steer', run80.rows);
  const graph = `${artifactDir}/step-steer-80kmh.svg`;
  writeLineChartSvg(graph, {
    title: '80 km/h step-steer — physical steering chain and chassis response',
    subtitle: 'Driver steering-wheel target held 1.5 s then released; rack/road wheel/tire/yaw threshold crossings use first fixed-step samples',
    xLabel: 'time (s)',
    yLabel: 'scaled response',
    markerX: 1.5,
    markerLabel: 'driver target release',
    x: run80.rows.map((row) => Number(row.time_s)),
    series: [
      { name: 'road-wheel deg', values: run80.rows.map((row) => (Number(row.fl_steer_deg) + Number(row.fr_steer_deg)) * 0.5) },
      { name: 'yaw deg/s', values: run80.rows.map((row) => Number(row.yaw_rate_deg_s)) },
      { name: 'lateral g × 30', values: run80.rows.map((row) => Number(row.lateral_g) * 30) },
    ],
  });

  return {
    id: 'step-steer',
    name: 'Step-steer physical steering-chain response and yaw-rate gain',
    status: coherent ? 'NO REFERENCE DATA' : 'FAIL',
    validationClass: 'internal-regression',
    blocking: !coherent,
    summary: `80 km/h road-wheel onset ${run80.steer10?.toFixed(3) ?? 'n/a'} s, yaw onset ${run80.yaw10?.toFixed(3) ?? 'n/a'} s, 10–90 yaw rise ${run80.riseTimeSec?.toFixed(3) ?? 'n/a'} s.`,
    metrics: Object.fromEntries(runs.flatMap((run) => [
      [`${run.speedKmh}Kmh_steeringWheel10Sec`, run.steeringWheel10],
      [`${run.speedKmh}Kmh_roadWheel10Sec`, run.steer10],
      [`${run.speedKmh}Kmh_frontSlip10Sec`, run.slip10],
      [`${run.speedKmh}Kmh_yaw10Sec`, run.yaw10],
      [`${run.speedKmh}Kmh_commandToRoadWheel10Sec`, run.commandToRoadWheel10Sec],
      [`${run.speedKmh}Kmh_commandToYaw10Sec`, run.commandToYaw10Sec],
      [`${run.speedKmh}Kmh_yawRiseTimeSec`, run.riseTimeSec],
      [`${run.speedKmh}Kmh_yawOvershootPercent`, run.overshoot],
      [`${run.speedKmh}Kmh_settlingTimeSec`, run.settlingTimeSec],
      [`${run.speedKmh}Kmh_yawRateGainDegSPerRoadWheelDeg`, run.yawRateGainRoadWheel],
      [`${run.speedKmh}Kmh_yawRateGainDegSPerSteeringWheelDeg`, run.yawRateGainSteeringWheel],
      [`${run.speedKmh}Kmh_roadWheelAngleDegPerG`, run.roadWheelAnglePerG],
      [`${run.speedKmh}Kmh_steeringWheelAngleDegPerG`, run.steeringWheelAnglePerG],
      [`${run.speedKmh}Kmh_chronologyCorrect`, run.chronologyCorrect ? 1 : 0],
    ])),
    diagnostics: [
      ...(!coherent ? [
        'Steering-wheel/rack/road-wheel/slip/yaw chronology is physically incoherent or the response diverged. Inspect the steering rack, tire relaxation and chassis force path before tuning.',
      ] : []),
      'Physical steering-wheel and rack telemetry is measured directly; no 14.2× road-wheel estimate is used in this test.',
      'REFERENCE DATA NEEDED for instrumented G90 steering-wheel angle vs lateral g, road-wheel angle vs lateral g, step response delay/rise/overshoot and yaw-rate gain.',
    ],
    telemetryFile,
    graphFiles: [graph],
  };
}
