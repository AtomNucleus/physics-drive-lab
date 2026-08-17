import { writeFileSync } from 'node:fs';
import { Simulation } from '../Simulation';
import { PhysicsMath } from '../math/PhysicsMath';
import { DEFAULT_VEHICLE_CONFIG } from '../vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import { ValidationSurfaceProvider } from './ValidationSurfaceProvider';
import { findM5Reference } from './M5ReferenceData';
import { ensureArtifactDir, writeLineChartSvg, writeRowsCsv } from './ValidationArtifacts';

const G = 9.81;
const DT = 1 / 120;
const RAD_TO_DEG = 180 / Math.PI;
const DEG_TO_RAD = Math.PI / 180;
const MPH_TO_KMH = 1.609344;
const M_TO_FT = 3.280839895;

const CONFIG = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES } as any;
const NEUTRAL = { throttle: 0, brake: 0, steer: 0, handbrake: false, shiftUp: false, shiftDown: false };

type Status = 'PASS' | 'WARNING' | 'FAIL' | 'NO REFERENCE DATA';
export type CorrectedValidationResult = {
  id: string;
  name: string;
  status: Status;
  validationClass: 'hard' | 'engineering-plausibility' | 'internal-regression';
  blocking: boolean;
  summary: string;
  metrics: Record<string, number | string | null>;
  diagnostics: string[];
  reference?: any;
  telemetryFile?: string;
  graphFiles?: string[];
};

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const mean = (v: number[]) => v.length ? v.reduce((a, b) => a + b, 0) / v.length : 0;
const maxAbs = (v: number[]) => v.length ? Math.max(...v.map(Math.abs)) : 0;
const wrap = (v: number) => Math.atan2(Math.sin(v), Math.cos(v));

function statusFor(metric: string, value: number) {
  const reference = findM5Reference(metric);
  if (!reference || !Number.isFinite(value)) return { status: 'NO REFERENCE DATA' as Status, reference };
  const min = reference.min ?? reference.target;
  const max = reference.max ?? reference.target;
  const target = reference.target ?? ((min ?? 0) + (max ?? 0)) * 0.5;
  if (!Number.isFinite(min) || !Number.isFinite(max)) return { status: 'NO REFERENCE DATA' as Status, reference };
  const errorPercent = target ? ((value - target) / target) * 100 : undefined;
  if (value >= min! && value <= max!) return { status: 'PASS' as Status, reference, errorPercent };
  const span = Math.max(Math.abs(max! - min!), Math.abs(target) * 0.01, 1e-6);
  if (value >= min! - 0.5 * span && value <= max! + 0.5 * span) return { status: 'WARNING' as Status, reference, errorPercent };
  return { status: 'FAIL' as Status, reference, errorPercent };
}

function combine(statuses: Status[]): Status {
  if (statuses.includes('FAIL')) return 'FAIL';
  if (statuses.includes('WARNING')) return 'WARNING';
  if (statuses.every((s) => s === 'NO REFERENCE DATA')) return 'NO REFERENCE DATA';
  if (statuses.includes('PASS')) return 'PASS';
  return 'NO REFERENCE DATA';
}

function makeSim(surface = new ValidationSurfaceProvider({ friction: 1.0 })) {
  const sim = new Simulation(CONFIG, surface);
  sim.reset(0, 0, 0);
  for (let i = 0; i < 264; i++) sim.stepExplicit(NEUTRAL as any, 1);
  return sim;
}

function setSpeed(sim: Simulation, speedMs: number) {
  sim.vehicle.rigidBody.velocity = PhysicsMath.quatRotateVec3(
    sim.vehicle.rigidBody.orientation,
    PhysicsMath.vec3(0, 0, speedMs)
  );
  sim.vehicle.wheels.forEach((w) => w.reset(speedMs));
}

function autoShift(sim: Simulation) {
  const state = sim.vehicle.getState() as any;
  if (state.rpm > CONFIG.revLimiterRpm * 0.94 && state.gear > 0 && state.gear < 8) {
    sim.vehicle.powertrain.shiftUp();
  }
}

function basicRow(sim: Simulation, t: number, controls: any) {
  const state = sim.vehicle.getState() as any;
  const rb = sim.vehicle.rigidBody;
  const local = rb.getLocalVelocity();
  const row: Record<string, unknown> = {
    time_s: t,
    speed_kmh: state.speedKmh,
    x_m: state.x,
    y_m: state.y,
    z_m: state.z,
    vx_body_ms: local.x,
    vy_body_ms: local.y,
    vz_body_ms: local.z,
    lateral_g: state.lateralG,
    longitudinal_g: state.longitudinalG,
    vertical_g: state.verticalG,
    yaw_deg: state.yaw * RAD_TO_DEG,
    pitch_deg: state.pitch * RAD_TO_DEG,
    roll_deg: state.roll * RAD_TO_DEG,
    yaw_rate_deg_s: state.yawRate * RAD_TO_DEG,
    yaw_accel_deg_s2: rb.angularAcceleration.y * RAD_TO_DEG,
    sideslip_deg: Math.atan2(local.x, Math.max(0.1, Math.abs(local.z))) * RAD_TO_DEG,
    throttle: controls.throttle,
    brake: controls.brake,
    steer_command: controls.steer,
    gear: state.gear,
    rpm: state.rpm,
    abs_active: state.absActive,
    tcs_active: state.tcsActive,
  };
  state.wheels.forEach((w: any, i: number) => {
    const p = ['fl', 'fr', 'rl', 'rr'][i];
    const susp = sim.vehicle.suspension.states[i] as any;
    row[`${p}_fz_n`] = w.forceVectorNorm;
    row[`${p}_fx_n`] = w.forceVectorLong;
    row[`${p}_fy_n`] = w.forceVectorLat;
    row[`${p}_slip_angle_deg`] = w.slipAngle * RAD_TO_DEG;
    row[`${p}_slip_ratio`] = w.slipRatio;
    row[`${p}_steer_deg`] = w.steerAngle * RAD_TO_DEG;
    row[`${p}_suspension_displacement_m`] = susp.displacement;
    row[`${p}_hub_world_y_m`] = susp.hubPositionWorldY;
    row[`${p}_hub_velocity_ms`] = susp.hubVelocityWorldY;
    row[`${p}_unsprung_accel_ms2`] = susp.unsprungAccelerationMps2;
    row[`${p}_spring_force_n`] = susp.springForceN;
    row[`${p}_damper_force_n`] = susp.damperForceN;
    row[`${p}_bumpstop_force_n`] = susp.bumpStopForceN;
    row[`${p}_grip_utilization`] = w.gripUtilization;
  });
  return row;
}

function writeTelemetry(artifactDir: string, id: string, rows: Record<string, unknown>[]) {
  ensureArtifactDir(`${artifactDir}/telemetry`);
  const path = `${artifactDir}/telemetry/${id}.csv`;
  writeRowsCsv(path, rows);
  return path;
}

function accelerateTo(sim: Simulation, targetKmh: number, timeoutSec = 16): number {
  const maxSteps = Math.round(timeoutSec / DT);
  for (let i = 0; i < maxSteps; i++) {
    sim.stepExplicit({ ...NEUTRAL, throttle: 1 } as any, 1);
    autoShift(sim);
    const speed = (sim.vehicle.getState() as any).speedKmh;
    if (speed >= targetKmh) return speed;
  }
  return (sim.vehicle.getState() as any).speedKmh;
}

function runBrakeCase(targetStartKmh: number) {
  const sim = makeSim();
  const reachedKmh = accelerateTo(sim, targetStartKmh);
  const rows: Record<string, unknown>[] = [];
  const startPos = { ...sim.vehicle.rigidBody.position };
  let prev = { ...startPos };
  let distance = 0;
  let peakDecelG = 0;
  let absFrames = 0;
  let peakFrontLoad = 0;
  let minRearLoad = Infinity;
  let peakPitchDeg = 0;
  const maxSteps = Math.round(8 / DT);

  for (let i = 0; i < maxSteps; i++) {
    const controls = { ...NEUTRAL, brake: 1 };
    const state = sim.stepExplicit(controls as any, 1) as any;
    const pos = sim.vehicle.rigidBody.position;
    distance += Math.hypot(pos.x - prev.x, pos.z - prev.z);
    prev = { ...pos };
    const t = (i + 1) * DT;
    rows.push(basicRow(sim, t, controls));
    peakDecelG = Math.max(peakDecelG, Math.max(0, -state.longitudinalG));
    if (state.absActive) absFrames++;
    const front = state.wheels[0].forceVectorNorm + state.wheels[1].forceVectorNorm;
    const rear = state.wheels[2].forceVectorNorm + state.wheels[3].forceVectorNorm;
    peakFrontLoad = Math.max(peakFrontLoad, front);
    minRearLoad = Math.min(minRearLoad, rear);
    peakPitchDeg = Math.max(peakPitchDeg, Math.abs(state.pitch * RAD_TO_DEG));
    if (state.speedKmh <= 1) break;
  }

  const stopTime = rows.length * DT;
  const startMs = reachedKmh / 3.6;
  return {
    reachedKmh, distance, stopTime, peakDecelG,
    averageDecelMs2: startMs / Math.max(stopTime, DT),
    absFraction: absFrames / Math.max(1, rows.length),
    peakFrontLoad, minRearLoad, peakPitchDeg, rows,
  };
}

function correctedBraking(artifactDir: string): CorrectedValidationResult {
  const kmh100 = runBrakeCase(100);
  const mph70 = runBrakeCase(70 * MPH_TO_KMH);
  const mph100 = runBrakeCase(100 * MPH_TO_KMH);
  const feet70 = mph70.distance * M_TO_FT;
  const feet100 = mph100.distance * M_TO_FT;
  const ref70 = statusFor('braking70To0MphFt', feet70);
  const ref100 = statusFor('braking100To0MphFt', feet100);
  const telemetryFile = writeTelemetry(artifactDir, 'braking', kmh100.rows);
  const graph = `${artifactDir}/braking-100kmh.svg`;
  writeLineChartSvg(graph, {
    title: '100–0 km/h braking — warmed driveline start',
    subtitle: 'Vehicle accelerates through the normal powertrain before braking begins',
    xLabel: 'time (s)', yLabel: 'scaled value',
    x: kmh100.rows.map((r) => Number(r.time_s)),
    series: [
      { name: 'speed km/h', values: kmh100.rows.map((r) => Number(r.speed_kmh)) },
      { name: 'decel g × 50', values: kmh100.rows.map((r) => -Number(r.longitudinal_g) * 50) },
    ],
  });
  const status = combine([ref70.status, ref100.status]);
  return {
    id: 'braking', name: 'Braking validation: 100–0 km/h, 70–0 mph and 100–0 mph',
    status, validationClass: 'hard', blocking: false,
    summary: `100–0 km/h ${kmh100.distance.toFixed(2)} m; 70–0 mph ${feet70.toFixed(1)} ft; 100–0 mph ${feet100.toFixed(1)} ft.`,
    metrics: {
      braking100To0KmhM: kmh100.distance,
      braking100To0KmhSec: kmh100.stopTime,
      braking100To0KmhActualStartKmh: kmh100.reachedKmh,
      braking100To0KmhPeakDecelG: kmh100.peakDecelG,
      braking100To0KmhAverageDecelMs2: kmh100.averageDecelMs2,
      braking70To0MphFt: feet70,
      braking70To0MphActualStartKmh: mph70.reachedKmh,
      braking100To0MphFt: feet100,
      braking100To0MphActualStartKmh: mph100.reachedKmh,
      absActiveFraction100Kmh: kmh100.absFraction,
      peakBrakePitchDeg: kmh100.peakPitchDeg,
      frontLoadPeakN: kmh100.peakFrontLoad,
      rearLoadMinimumN: kmh100.minRearLoad,
      braking70ErrorPercent: ref70.errorPercent ?? null,
      braking100MphErrorPercent: ref100.errorPercent ?? null,
    },
    diagnostics: status === 'FAIL' ? [
      'The result remains long after establishing a physically consistent driveline/gear state before braking.',
      'Investigate brake torque delivery, ABS slip regulation, tire longitudinal force/slip behavior, surface calibration and CG-driven load transfer before changing coefficients.',
      '100–0 km/h still has no directly comparable external G90 procedure in the reference table; 70–0 and 100–0 mph are the hard anchors.',
    ] : ['100–0 km/h remains descriptive until a directly comparable external G90 reference is found.'],
    reference: ref70.reference, telemetryFile, graphFiles: [graph],
  };
}

type CirclePoint = {
  targetG: number; measuredG: number; targetSpeedKmh: number; measuredSpeedKmh: number;
  measuredRadiusM: number; roadSteerDeg: number; steeringWheelDeg: number; rollDeg: number;
  sideslipDeg: number; loads: number[]; fys: number[]; slips: number[]; stable: boolean;
};

function runCircle(targetG: number, radiusM = 45.72): { point: CirclePoint; rows: Record<string, unknown>[] } {
  const surface = new ValidationSurfaceProvider({ friction: 1.0, rollingResistance: 0.015 });
  const sim = new Simulation(CONFIG, surface);
  sim.reset(-radiusM, 0, 0);
  for (let i = 0; i < 264; i++) sim.stepExplicit(NEUTRAL as any, 1);
  const targetSpeedMs = Math.sqrt(targetG * G * radiusM);
  setSpeed(sim, targetSpeedMs);
  const kinematicSteer = Math.atan(CONFIG.wheelbase / radiusM);
  const rows: Record<string, unknown>[] = [];
  const duration = 5.5;

  for (let i = 0; i < Math.round(duration / DT); i++) {
    const state = sim.vehicle.getState() as any;
    const x = sim.vehicle.rigidBody.position.x;
    const z = sim.vehicle.rigidBody.position.z;
    const rNow = Math.hypot(x, z);
    const desiredYaw = Math.atan2(z, -x);
    const headingError = wrap(desiredYaw - state.yaw);
    const radialError = (rNow - radiusM) / radiusM;
    const roadSteer = kinematicSteer + 1.35 * headingError + 0.85 * radialError;
    const steer = clamp(roadSteer / CONFIG.maxSteerAngle, -1, 1);
    const speedError = targetSpeedMs - state.speedMs;
    // This is only a scripted driver speed controller. It changes throttle/brake,
    // never vehicle velocity or force. Full throttle is available if tire scrub/drag
    // would otherwise prevent the requested test speed from being held.
    const throttle = clamp(0.16 + speedError * 0.42, 0, 1);
    const brake = clamp(-speedError * 0.22, 0, 0.7);
    const controls = { ...NEUTRAL, steer, throttle, brake };
    sim.stepExplicit(controls as any, 1);
    autoShift(sim);
    rows.push(basicRow(sim, (i + 1) * DT, controls));
  }

  const tail = rows.slice(-Math.round(1.2 / DT));
  const avg = (key: string) => mean(tail.map((r) => Number(r[key])));
  const measuredG = mean(tail.map((r) => Math.abs(Number(r.lateral_g))));
  const measuredSpeedMs = avg('speed_kmh') / 3.6;
  const yawRate = mean(tail.map((r) => Math.abs(Number(r.yaw_rate_deg_s) / RAD_TO_DEG)));
  const measuredRadiusM = yawRate > 1e-6 ? measuredSpeedMs / yawRate : Infinity;
  const roadSteerDeg = mean(tail.map((r) => Math.abs((Number(r.fl_steer_deg) + Number(r.fr_steer_deg)) * 0.5)));
  const steeringWheelDeg = roadSteerDeg * 14.2;
  const rollDeg = mean(tail.map((r) => Math.abs(Number(r.roll_deg))));
  const sideslipDeg = mean(tail.map((r) => Math.abs(Number(r.sideslip_deg))));
  const loads = ['fl','fr','rl','rr'].map((p) => avg(`${p}_fz_n`));
  const fys = ['fl','fr','rl','rr'].map((p) => mean(tail.map((r) => Math.abs(Number(r[`${p}_fy_n`)))));
  const slips = ['fl','fr','rl','rr'].map((p) => mean(tail.map((r) => Math.abs(Number(r[`${p}_slip_angle_deg`)))));
  const radiusError = Math.abs(measuredRadiusM - radiusM) / radiusM;
  const speedError = Math.abs(avg('speed_kmh') - targetSpeedMs * 3.6) / (targetSpeedMs * 3.6);
  const stable = radiusError < 0.08 && speedError < 0.08 && sideslipDeg < 8;
  return {
    point: {
      targetG, measuredG, targetSpeedKmh: targetSpeedMs * 3.6, measuredSpeedKmh: avg('speed_kmh'),
      measuredRadiusM, roadSteerDeg, steeringWheelDeg, rollDeg, sideslipDeg, loads, fys, slips, stable,
    }, rows,
  };
}

function slope(points: { x: number; y: number }[]) {
  if (points.length < 2) return Number.NaN;
  const mx = mean(points.map((p) => p.x));
  const my = mean(points.map((p) => p.y));
  const d = points.reduce((s, p) => s + (p.x - mx) ** 2, 0);
  return d > 1e-12 ? points.reduce((s, p) => s + (p.x - mx) * (p.y - my), 0) / d : Number.NaN;
}

function correctedSkidpad(artifactDir: string): CorrectedValidationResult {
  const targets = [0.10,0.20,0.30,0.40,0.50,0.60,0.70,0.80,0.90,0.98,1.05];
  const runs = targets.map((g) => runCircle(g));
  const stable = runs.map((r) => r.point).filter((p) => p.stable);
  const peak = stable.length ? Math.max(...stable.map((p) => p.measuredG)) : 0;
  const ref = statusFor('skidpadPeakG', peak);
  const gradientPts = stable.filter((p) => p.measuredG >= 0.18 && p.measuredG <= 0.75);
  const kinematicDeg = Math.atan(CONFIG.wheelbase / 45.72) * RAD_TO_DEG;
  const rollGradient = slope(gradientPts.map((p) => ({ x: p.measuredG, y: p.rollDeg })));
  const understeerGradient = slope(gradientPts.map((p) => ({ x: p.measuredG, y: p.roadSteerDeg - kinematicDeg })));
  const steeringWheelSlope = slope(gradientPts.map((p) => ({ x: p.measuredG, y: p.steeringWheelDeg })));
  const loadPoint = stable.length ? stable.reduce((best, p) => Math.abs(p.measuredG - 0.6) < Math.abs(best.measuredG - 0.6) ? p : best, stable[0]) : runs[0].point;
  const left = loadPoint.loads[0] + loadPoint.loads[2];
  const right = loadPoint.loads[1] + loadPoint.loads[3];
  const loadDirectionOk = right > left;
  const sweepRows = runs.map(({ point }) => ({
    radius_m: 45.72, target_lateral_g: point.targetG, measured_lateral_g: point.measuredG,
    target_speed_kmh: point.targetSpeedKmh, measured_speed_kmh: point.measuredSpeedKmh,
    measured_radius_m: point.measuredRadiusM, stable: point.stable,
    road_wheel_steer_deg: point.roadSteerDeg, steering_wheel_estimate_deg: point.steeringWheelDeg,
    roll_deg: point.rollDeg, sideslip_deg: point.sideslipDeg,
    fz_fl_n: point.loads[0], fz_fr_n: point.loads[1], fz_rl_n: point.loads[2], fz_rr_n: point.loads[3],
    fy_fl_n: point.fys[0], fy_fr_n: point.fys[1], fy_rl_n: point.fys[2], fy_rr_n: point.fys[3],
    slip_fl_deg: point.slips[0], slip_fr_deg: point.slips[1], slip_rl_deg: point.slips[2], slip_rr_deg: point.slips[3],
  }));
  const sweepFile = `${artifactDir}/skidpad-sweep.csv`;
  writeRowsCsv(sweepFile, sweepRows);
  const representative = runs.reduce((best, r) => Math.abs(r.point.targetG - 0.98) < Math.abs(best.point.targetG - 0.98) ? r : best, runs[0]);
  const telemetryFile = writeTelemetry(artifactDir, 'skidpad', representative.rows);
  const graph = `${artifactDir}/skidpad-steering-vs-lateral-g.svg`;
  writeLineChartSvg(graph, {
    title: '45.72 m skidpad — validated speed/radius hold',
    subtitle: 'Only points within ±8% speed and radius and <8° sideslip count toward peak grip',
    xLabel: 'lateral acceleration (g)', yLabel: 'angle (deg)',
    x: stable.map((p) => p.measuredG),
    series: [
      { name: 'road-wheel steer', values: stable.map((p) => p.roadSteerDeg) },
      { name: 'body roll', values: stable.map((p) => p.rollDeg) },
    ],
  });
  let status = ref.status;
  if (!loadDirectionOk) status = 'FAIL';
  if (!stable.length) status = 'FAIL';
  return {
    id: 'skidpad', name: 'Constant-radius skidpad, steering demand, understeer and load transfer',
    status, validationClass: 'hard', blocking: false,
    summary: stable.length ? `45.72 m validated sweep reached ${peak.toFixed(3)} g; ${stable.length}/${runs.length} points held speed/radius.` : 'No high-confidence fixed-radius point met the driver hold criteria.',
    metrics: {
      skidpadPeakG: peak,
      stablePointCount: stable.length,
      requestedPointCount: runs.length,
      rollGradientDegPerG: rollGradient,
      roadWheelUndersteerGradientDegPerG: understeerGradient,
      steeringWheelAngleSlopeDegPerG: steeringWheelSlope,
      loadCheckAtG: loadPoint.measuredG,
      FL_Fz_N: loadPoint.loads[0], FR_Fz_N: loadPoint.loads[1], RL_Fz_N: loadPoint.loads[2], RR_Fz_N: loadPoint.loads[3],
      outsideRightLoadN: right, insideLeftLoadN: left,
      outsideLoadTransferDirectionCorrect: loadDirectionOk ? 1 : 0,
    },
    diagnostics: [
      ...(ref.status === 'FAIL' ? ['If the corrected driver holds target speed/radius and grip is still low, inspect tire lateral force/load sensitivity, camber, steering geometry and front/rear load transfer.'] : []),
      ...(!loadDirectionOk ? ['Left-turn lateral load transfer direction is reversed; treat as a blocking sign/convention defect.'] : []),
      'Steering-wheel angle remains an explicitly labeled 14.2:1 estimate until the physical steering-rack branch is integrated.',
      'External G90 roll-gradient and understeer-gradient references are still needed.',
    ],
    reference: ref.reference, telemetryFile, graphFiles: [graph, sweepFile],
  };
}

function firstAtOrAbove(values: number[], threshold: number, times: number[]) {
  const idx = values.findIndex((v) => v >= threshold);
  return idx >= 0 ? times[idx] : null;
}

function runStep(speedKmh: number) {
  const sim = makeSim();
  setSpeed(sim, speedKmh / 3.6);
  const rows: Record<string, unknown>[] = [];
  const steer = (3 * DEG_TO_RAD) / CONFIG.maxSteerAngle;
  for (let i = 0; i < Math.round(3 / DT); i++) {
    const controls = { ...NEUTRAL, steer: i * DT < 1.5 ? steer : 0 };
    sim.stepExplicit(controls as any, 1);
    rows.push(basicRow(sim, i * DT, controls));
  }
  const hold = rows.filter((r) => Number(r.time_s) >= 0.8 && Number(r.time_s) < 1.5);
  const steadyYaw = mean(hold.map((r) => Math.abs(Number(r.yaw_rate_deg_s))));
  const steadySteer = mean(hold.map((r) => Math.abs((Number(r.fl_steer_deg) + Number(r.fr_steer_deg)) * 0.5)));
  const steadySlip = mean(hold.map((r) => 0.5 * (Math.abs(Number(r.fl_slip_angle_deg)) + Math.abs(Number(r.fr_slip_angle_deg)))));
  const times = rows.map((r) => Number(r.time_s));
  const steerSeries = rows.map((r) => Math.abs((Number(r.fl_steer_deg) + Number(r.fr_steer_deg)) * 0.5));
  const slipSeries = rows.map((r) => 0.5 * (Math.abs(Number(r.fl_slip_angle_deg)) + Math.abs(Number(r.fr_slip_angle_deg))));
  const yawSeries = rows.map((r) => Math.abs(Number(r.yaw_rate_deg_s)));
  const steer10 = firstAtOrAbove(steerSeries, steadySteer * 0.10, times);
  const slip10 = firstAtOrAbove(slipSeries, steadySlip * 0.10, times);
  const yaw10 = firstAtOrAbove(yawSeries, steadyYaw * 0.10, times);
  const yaw90 = firstAtOrAbove(yawSeries, steadyYaw * 0.90, times);
  const peakYaw = Math.max(...rows.filter((r) => Number(r.time_s) < 1.5).map((r) => Math.abs(Number(r.yaw_rate_deg_s))));
  const overshoot = steadyYaw > 1e-6 ? (peakYaw / steadyYaw - 1) * 100 : null;
  const releaseRows = rows.filter((r) => Number(r.time_s) >= 1.5);
  const settleThreshold = steadyYaw * 0.05;
  const window = Math.round(0.2 / DT);
  let settle: number | null = null;
  for (let i = 0; i <= releaseRows.length - window; i++) {
    if (releaseRows.slice(i, i + window).every((r) => Math.abs(Number(r.yaw_rate_deg_s)) <= settleThreshold)) {
      settle = Number(releaseRows[i].time_s) - 1.5;
      break;
    }
  }
  const yawGain = steadySteer > 1e-6 ? steadyYaw / steadySteer : null;
  const chronology = steer10 !== null && slip10 !== null && yaw10 !== null && yaw10 >= slip10 + 0.5 * DT;
  return {
    speedKmh, rows, steadyYaw, steadySteer, steadySlip, steer10, slip10, yaw10, yaw90,
    rise: yaw10 !== null && yaw90 !== null ? yaw90 - yaw10 : null,
    overshoot, settle, yawGain, chronology,
  };
}

function correctedStepSteer(artifactDir: string): CorrectedValidationResult {
  const runs = [30,50,80,100].map(runStep);
  const r80 = runs.find((r) => r.speedKmh === 80)!;
  const coherent = runs.every((r) => r.chronology && (r.overshoot ?? 999) < 120 && maxAbs(r.rows.map((x) => Number(x.roll_deg))) < 12);
  const telemetryFile = writeTelemetry(artifactDir, 'step-steer', r80.rows);
  const graph = `${artifactDir}/step-steer-80kmh.svg`;
  writeLineChartSvg(graph, {
    title: '80 km/h step-steer — corrected onset measurement',
    subtitle: 'First-sample threshold crossings are treated as t=0 instead of being discarded',
    xLabel: 'time (s)', yLabel: 'scaled response', markerX: 1.5, markerLabel: 'steering release',
    x: r80.rows.map((r) => Number(r.time_s)),
    series: [
      { name: 'yaw deg/s', values: r80.rows.map((r) => Number(r.yaw_rate_deg_s)) },
      { name: 'roll deg × 10', values: r80.rows.map((r) => Number(r.roll_deg) * 10) },
      { name: 'lateral g × 30', values: r80.rows.map((r) => Number(r.lateral_g) * 30) },
    ],
  });
  return {
    id: 'step-steer', name: 'Step-steer response and yaw-rate gain',
    status: coherent ? 'NO REFERENCE DATA' : 'FAIL', validationClass: 'internal-regression', blocking: !coherent,
    summary: `80 km/h yaw onset ${r80.yaw10?.toFixed(3) ?? 'n/a'} s, 10–90 rise ${r80.rise?.toFixed(3) ?? 'n/a'} s, overshoot ${r80.overshoot?.toFixed(1) ?? 'n/a'}%.`,
    metrics: Object.fromEntries(runs.flatMap((r) => [
      [`${r.speedKmh}Kmh_steer10Sec`, r.steer10], [`${r.speedKmh}Kmh_frontSlip10Sec`, r.slip10],
      [`${r.speedKmh}Kmh_yaw10Sec`, r.yaw10], [`${r.speedKmh}Kmh_yawRiseTimeSec`, r.rise],
      [`${r.speedKmh}Kmh_yawOvershootPercent`, r.overshoot], [`${r.speedKmh}Kmh_settlingTimeSec`, r.settle],
      [`${r.speedKmh}Kmh_yawRateGainDegSPerRoadWheelDeg`, r.yawGain], [`${r.speedKmh}Kmh_chronologyCorrect`, r.chronology ? 1 : 0],
    ])),
    diagnostics: [
      ...(coherent ? [] : ['Steer/slip/yaw chronology is physically incoherent or the response diverged; inspect steering response, tire relaxation, force buildup and inertia.']),
      'The current CG/inertia base branch does not yet include PR #26 physical rack dynamics, so road-wheel steering begins within the first fixed step.',
      'REFERENCE DATA NEEDED for instrumented G90 step-steer delay, rise time, overshoot and yaw-rate gain.',
    ], telemetryFile, graphFiles: [graph],
  };
}

function dominantFrequency(values: number[], minHz: number, maxHz: number) {
  if (values.length < 60) return null;
  const m = mean(values);
  const v = values.map((x) => x - m);
  let bestHz = minHz;
  let bestPower = -1;
  for (let hz = minHz; hz <= maxHz + 1e-9; hz += 0.25) {
    let re = 0, im = 0;
    for (let i = 0; i < v.length; i++) {
      const phase = 2 * Math.PI * hz * i * DT;
      re += v[i] * Math.cos(phase);
      im -= v[i] * Math.sin(phase);
    }
    const p = re * re + im * im;
    if (p > bestPower) { bestPower = p; bestHz = hz; }
  }
  return bestPower > 1e-16 ? bestHz : null;
}

function runBump(kind: 'bump-left' | 'bump-full', axle: 'front' | 'rear') {
  const surface = new ValidationSurfaceProvider({ kind, bumpStartZ: 20, bumpLengthM: 0.55, bumpHeightM: 0.025, friction: 1.0 });
  const sim = new Simulation(CONFIG, surface);
  const props = sim.vehicle.chassisMassProperties;
  const startZ = axle === 'front' ? 20 - props.cgToFrontAxle - 1.0 : 20 + props.cgToRearAxle - 0.9;
  sim.reset(0, startZ, 0);
  for (let i = 0; i < 300; i++) sim.stepExplicit(NEUTRAL as any, 1);
  setSpeed(sim, 30 / 3.6);
  const wheelIndex = axle === 'front' ? 0 : 2;
  const rows: Record<string, unknown>[] = [];
  for (let i = 0; i < Math.round(2.5 / DT); i++) {
    const controls = { ...NEUTRAL, throttle: 0.08 };
    sim.stepExplicit(controls as any, 1);
    const row = basicRow(sim, i * DT, controls);
    const contact = sim.vehicle.suspension.states[wheelIndex].contactPointWorld;
    row.road_elevation_m = surface.sampleSurface(contact.x, contact.z).elevation;
    rows.push(row);
  }
  const p = wheelIndex === 0 ? 'fl' : 'rl';
  const roadOnsetIdx = rows.findIndex((r) => Number(r.road_elevation_m) > 0.0002);
  const onset = roadOnsetIdx >= 0 ? roadOnsetIdx : 0;
  const baselineStart = Math.max(0, onset - 12);
  const baselineRows = rows.slice(baselineStart, onset || 1);
  const baseHub = mean(baselineRows.map((r) => Number(r[`${p}_hub_world_y_m`])));
  const baseBody = mean(baselineRows.map((r) => Number(r.y_m)));
  const hubDelta = rows.map((r) => Number(r[`${p}_hub_world_y_m`]) - baseHub);
  const bodyDelta = rows.map((r) => Number(r.y_m) - baseBody);
  const findResponse = (signal: number[], threshold: number) => {
    for (let i = onset; i < signal.length; i++) if (Math.abs(signal[i]) >= threshold) return Number(rows[i].time_s);
    return null;
  };
  const roadOnsetSec = Number(rows[onset]?.time_s ?? 0);
  const hubResponseSec = findResponse(hubDelta, 0.0005);
  const bodyResponseSec = findResponse(bodyDelta, 0.0003);
  const roadEndIdx = rows.findIndex((r, i) => i > onset && Number(r.road_elevation_m) <= 0.0002);
  const spectralStart = Math.min(rows.length - 60, Math.max(onset + 1, (roadEndIdx > 0 ? roadEndIdx : onset) + Math.round(0.04 / DT)));
  const hubSpectral = hubDelta.slice(spectralStart);
  const bodySpectral = bodyDelta.slice(spectralStart);
  return {
    rows, p, roadOnsetSec, hubResponseSec, bodyResponseSec,
    hubDelaySec: hubResponseSec === null ? null : hubResponseSec - roadOnsetSec,
    bodyDelaySec: bodyResponseSec === null ? null : bodyResponseSec - roadOnsetSec,
    wheelHopHz: dominantFrequency(hubSpectral, 5, 25),
    bodyHeaveHz: dominantFrequency(bodySpectral, 0.5, 5),
    hubPeakM: maxAbs(hubDelta), bodyPeakM: maxAbs(bodyDelta),
    hubDelta, bodyDelta,
  };
}

function correctedBump(artifactDir: string): CorrectedValidationResult {
  const front = runBump('bump-left', 'front');
  const rear = runBump('bump-left', 'rear');
  const full = runBump('bump-full', 'front');
  const sequence = [front, rear].every((r) => r.hubResponseSec !== null && r.bodyResponseSec !== null && r.hubResponseSec <= r.bodyResponseSec + 0.5 * DT);
  const telemetryFile = writeTelemetry(artifactDir, 'bump-response', front.rows);
  const graph = `${artifactDir}/bump-response.svg`;
  writeLineChartSvg(graph, {
    title: 'Single-front-wheel bump — unsprung hub vs chassis',
    subtitle: 'Response timing is measured only after the tire actually reaches the road bump',
    xLabel: 'time (s)', yLabel: 'vertical displacement from pre-bump baseline (m)',
    x: front.rows.map((r) => Number(r.time_s)),
    series: [
      { name: 'FL hub', values: front.hubDelta },
      { name: 'chassis CG', values: front.bodyDelta },
    ], markerX: front.roadOnsetSec, markerLabel: 'road contact',
  });
  return {
    id: 'bump-response', name: 'Single-wheel/full-width bump and wheel-hop response',
    status: sequence ? 'NO REFERENCE DATA' : 'FAIL', validationClass: 'engineering-plausibility', blocking: !sequence,
    summary: `Front hub ${front.hubDelaySec?.toFixed(3) ?? 'n/a'} s after road onset; chassis ${front.bodyDelaySec?.toFixed(3) ?? 'n/a'} s. Hub frequency ${front.wheelHopHz?.toFixed(2) ?? 'n/a'} Hz.`,
    metrics: {
      frontRoadOnsetSec: front.roadOnsetSec,
      frontHubResponseDelaySec: front.hubDelaySec,
      frontBodyResponseDelaySec: front.bodyDelaySec,
      frontWheelHopHz: front.wheelHopHz,
      frontBodyHeaveHz: front.bodyHeaveHz,
      rearHubResponseDelaySec: rear.hubDelaySec,
      rearBodyResponseDelaySec: rear.bodyDelaySec,
      rearWheelHopHz: rear.wheelHopHz,
      rearBodyHeaveHz: rear.bodyHeaveHz,
      fullWidthWheelHopHz: full.wheelHopHz,
      frontHubPeakVerticalM: front.hubPeakM,
      frontBodyPeakHeaveM: front.bodyPeakM,
      wheelBeforeBodySequenceCorrect: sequence ? 1 : 0,
    },
    diagnostics: [
      ...(!sequence ? ['Road input is reaching the chassis before/with the unsprung hub response. Inspect the full Vehicle vertical force path; an internal tire-normal reaction may still be bypassing the spring/damper path.'] : []),
      'REFERENCE DATA NEEDED for production G90 wheel-hop, heave frequency and damping ratio.',
    ], telemetryFile, graphFiles: [graph],
  };
}

function totalKineticEnergy(sim: Simulation) {
  const rb = sim.vehicle.rigidBody;
  const I = rb.config.inertia;
  const omega = rb.getLocalAngularVelocity();
  const translational = 0.5 * rb.config.mass * PhysicsMath.vec3Dot(rb.velocity, rb.velocity);
  const rotational = 0.5 * (I.x * omega.x ** 2 + I.y * omega.y ** 2 + I.z * omega.z ** 2);
  const wheel = sim.vehicle.wheels.reduce((sum, w: any) => sum + 0.5 * CONFIG.wheelInertia * w.angularVelocity ** 2, 0);
  return translational + rotational + wheel;
}

function correctedEnergy(artifactDir: string): CorrectedValidationResult {
  const coast = makeSim();
  setSpeed(coast, 30 / 3.6);
  const e0 = totalKineticEnergy(coast);
  let maxEnergy = e0;
  const coastRows: Record<string, unknown>[] = [];
  for (let i = 0; i < Math.round(5 / DT); i++) {
    coast.stepExplicit(NEUTRAL as any, 1);
    maxEnergy = Math.max(maxEnergy, totalKineticEnergy(coast));
    coastRows.push(basicRow(coast, i * DT, NEUTRAL));
  }
  const coastFinal = (coast.vehicle.getState() as any).speedKmh;

  const turn = makeSim();
  setSpeed(turn, 20 / 3.6);
  const turnRows: Record<string, unknown>[] = [];
  const steer = (12 * DEG_TO_RAD) / CONFIG.maxSteerAngle;
  for (let i = 0; i < Math.round(5 / DT); i++) {
    const controls = { ...NEUTRAL, steer };
    turn.stepExplicit(controls as any, 1);
    turnRows.push(basicRow(turn, i * DT, controls));
  }
  const peakTurn = Math.max(...turnRows.map((r) => Number(r.speed_kmh)));
  const finalTurn = Number(turnRows[turnRows.length - 1].speed_kmh);

  const rest = makeSim();
  const restRows: Record<string, unknown>[] = [];
  for (let i = 0; i < Math.round(2 / DT); i++) {
    const controls = { ...NEUTRAL, steer };
    rest.stepExplicit(controls as any, 1);
    restRows.push(basicRow(rest, i * DT, controls));
  }
  const maxRestYaw = maxAbs(restRows.map((r) => Number(r.yaw_rate_deg_s)));
  const maxRestSpeed = Math.max(...restRows.map((r) => Number(r.speed_kmh)));
  const growth = e0 > 0 ? (maxEnergy / e0 - 1) * 100 : 0;
  const pass = growth < 0.5 && coastFinal <= 30.05 && peakTurn <= 20.6 && finalTurn <= 20.2 && maxRestYaw < 0.25 && maxRestSpeed < 0.2;
  const telemetryFile = writeTelemetry(artifactDir, 'energy-sanity', turnRows);
  return {
    id: 'energy-sanity', name: 'Energy, coast-down and low-speed turning sanity checks',
    status: pass ? 'PASS' : 'FAIL', validationClass: 'internal-regression', blocking: !pass,
    summary: `No-throttle 20 km/h turn peak ${peakTurn.toFixed(2)} km/h; true per-step coast energy growth ${growth.toFixed(3)}%.`,
    metrics: {
      coastInitialEnergyJ: e0, coastMaxEnergyGrowthPercent: growth, coastFinalSpeedKmh: coastFinal,
      lowSpeedTurnInitialKmh: 20, lowSpeedTurnPeakKmh: peakTurn, lowSpeedTurnFinalKmh: finalTurn,
      restSteeringPeakYawRateDegS: maxRestYaw, restSteeringPeakSpeedKmh: maxRestSpeed,
    },
    diagnostics: pass ? [] : ['Investigate tire-force direction, drivetrain feedback, rolling resistance and low-speed tire regularization; do not mask energy creation with damping.'],
    telemetryFile, graphFiles: [],
  };
}

export function runCorrectedValidationTests(artifactDir: string): CorrectedValidationResult[] {
  ensureArtifactDir(artifactDir);
  const tests = [correctedBraking, correctedSkidpad, correctedStepSteer, correctedBump, correctedEnergy];
  const results: CorrectedValidationResult[] = [];
  for (const test of tests) {
    const result = test(artifactDir);
    console.log(`[M5 validation corrected] ${result.id}: ${result.status} — ${result.summary}`);
    result.diagnostics.forEach((d) => console.log(`  - ${d}`));
    results.push(result);
  }
  writeFileSync(`${artifactDir}/corrected-results.json`, `${JSON.stringify(results, null, 2)}\n`, 'utf8');
  return results;
}
