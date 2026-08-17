import { writeLineChartSvg } from './ValidationArtifacts';
import {
  DT, MPH_TO_KMH, M_TO_FT, NEUTRAL, autoShift, basicRow, combineStatuses,
  accelerateTo, makeSim, statusFor, writeTelemetry, type CorrectedValidationResult,
} from './CorrectedValidationCommon';

function runBrakeCase(targetStartKmh: number) {
  const sim = makeSim();
  const reachedKmh = accelerateTo(sim, targetStartKmh);
  const rows: Record<string, unknown>[] = [];
  let previous = { ...sim.vehicle.rigidBody.position };
  let distanceM = 0;
  let peakDecelG = 0;
  let absFrames = 0;
  let peakFrontLoadN = 0;
  let minRearLoadN = Number.POSITIVE_INFINITY;
  let peakPitchDeg = 0;

  for (let i = 0; i < Math.round(8 / DT); i++) {
    const controls = { ...NEUTRAL, brake: 1 };
    const state = sim.stepExplicit(controls as any, 1) as any;
    autoShift(sim);
    const position = sim.vehicle.rigidBody.position;
    distanceM += Math.hypot(position.x - previous.x, position.z - previous.z);
    previous = { ...position };
    rows.push(basicRow(sim, (i + 1) * DT, controls));
    peakDecelG = Math.max(peakDecelG, Math.max(0, -state.longitudinalG));
    if (state.absActive) absFrames++;
    peakFrontLoadN = Math.max(peakFrontLoadN, state.wheels[0].forceVectorNorm + state.wheels[1].forceVectorNorm);
    minRearLoadN = Math.min(minRearLoadN, state.wheels[2].forceVectorNorm + state.wheels[3].forceVectorNorm);
    peakPitchDeg = Math.max(peakPitchDeg, Math.abs(state.pitch * 180 / Math.PI));
    if (state.speedKmh <= 1) break;
  }

  const stopTimeSec = rows.length * DT;
  return {
    reachedKmh,
    distanceM,
    stopTimeSec,
    peakDecelG,
    averageDecelMs2: (reachedKmh / 3.6) / Math.max(stopTimeSec, DT),
    absFraction: absFrames / Math.max(1, rows.length),
    peakFrontLoadN,
    minRearLoadN,
    peakPitchDeg,
    rows,
  };
}

export function runBrakingValidation(artifactDir: string): CorrectedValidationResult {
  const kmh100 = runBrakeCase(100);
  const mph70 = runBrakeCase(70 * MPH_TO_KMH);
  const mph100 = runBrakeCase(100 * MPH_TO_KMH);
  const feet70 = mph70.distanceM * M_TO_FT;
  const feet100 = mph100.distanceM * M_TO_FT;
  const ref70 = statusFor('braking70To0MphFt', feet70);
  const ref100 = statusFor('braking100To0MphFt', feet100);
  const status = combineStatuses([ref70.status, ref100.status]);
  const telemetryFile = writeTelemetry(artifactDir, 'braking', kmh100.rows);
  const graph = `${artifactDir}/braking-100kmh.svg`;
  writeLineChartSvg(graph, {
    title: '100–0 km/h braking — warmed driveline start',
    subtitle: 'Vehicle accelerates through the normal powertrain before braking begins',
    xLabel: 'time (s)',
    yLabel: 'scaled value',
    x: kmh100.rows.map((row) => Number(row.time_s)),
    series: [
      { name: 'speed km/h', values: kmh100.rows.map((row) => Number(row.speed_kmh)) },
      { name: 'decel g × 50', values: kmh100.rows.map((row) => -Number(row.longitudinal_g) * 50) },
    ],
  });

  return {
    id: 'braking',
    name: 'Braking validation: 100–0 km/h, 70–0 mph and 100–0 mph',
    status,
    validationClass: 'hard',
    blocking: false,
    summary: `100–0 km/h ${kmh100.distanceM.toFixed(2)} m; 70–0 mph ${feet70.toFixed(1)} ft; 100–0 mph ${feet100.toFixed(1)} ft.`,
    metrics: {
      braking100To0KmhM: kmh100.distanceM,
      braking100To0KmhSec: kmh100.stopTimeSec,
      braking100To0KmhActualStartKmh: kmh100.reachedKmh,
      braking100To0KmhPeakDecelG: kmh100.peakDecelG,
      braking100To0KmhAverageDecelMs2: kmh100.averageDecelMs2,
      braking70To0MphFt: feet70,
      braking70To0MphActualStartKmh: mph70.reachedKmh,
      braking100To0MphFt: feet100,
      braking100To0MphActualStartKmh: mph100.reachedKmh,
      absActiveFraction100Kmh: kmh100.absFraction,
      peakBrakePitchDeg: kmh100.peakPitchDeg,
      frontLoadPeakN: kmh100.peakFrontLoadN,
      rearLoadMinimumN: kmh100.minRearLoadN,
      braking70ErrorPercent: ref70.errorPercent ?? null,
      braking100MphErrorPercent: ref100.errorPercent ?? null,
    },
    diagnostics: status === 'FAIL' ? [
      'The result remains outside the Car and Driver braking references after establishing a normal driveline/gear state before brake application.',
      'Investigate brake torque delivery, ABS slip regulation, longitudinal tire force/slip behavior, test-surface friction and CG-driven load transfer before changing calibration.',
      '100–0 km/h remains descriptive until a directly comparable external G90 procedure is found.',
    ] : ['100–0 km/h remains descriptive until a directly comparable external G90 procedure is found.'],
    reference: ref70.reference,
    telemetryFile,
    graphFiles: [graph],
  };
}
