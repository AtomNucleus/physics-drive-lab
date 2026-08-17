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
  let stopped = false;
  let minSpeedKmh = reachedKmh;
  let positiveAccelFramesBelow15Kmh = 0;
  let rearPositiveFxFramesBelow15Kmh = 0;

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
    minSpeedKmh = Math.min(minSpeedKmh, state.speedKmh);

    if (state.speedKmh < 15) {
      if (state.longitudinalG > 0.02) positiveAccelFramesBelow15Kmh++;
      if (state.wheels[2].forceVectorLong + state.wheels[3].forceVectorLong > 500) {
        rearPositiveFxFramesBelow15Kmh++;
      }
    }

    if (state.speedKmh <= 1) {
      stopped = true;
      break;
    }
  }

  const elapsedSec = rows.length * DT;
  const finalSpeedKmh = Number(rows.at(-1)?.speed_kmh ?? reachedKmh);
  return {
    reachedKmh,
    distanceM,
    elapsedSec,
    stopped,
    finalSpeedKmh,
    minSpeedKmh,
    peakDecelG,
    averageDecelMs2: stopped ? (reachedKmh / 3.6) / Math.max(elapsedSec, DT) : null,
    absFraction: absFrames / Math.max(1, rows.length),
    peakFrontLoadN,
    minRearLoadN,
    peakPitchDeg,
    positiveAccelFramesBelow15Kmh,
    rearPositiveFxFramesBelow15Kmh,
    rows,
  };
}

export function runBrakingValidation(artifactDir: string): CorrectedValidationResult {
  const kmh100 = runBrakeCase(100);
  const mph70 = runBrakeCase(70 * MPH_TO_KMH);
  const mph100 = runBrakeCase(100 * MPH_TO_KMH);
  const allStopped = kmh100.stopped && mph70.stopped && mph100.stopped;

  // A run that never reaches <=1 km/h is not a stopping-distance measurement.
  // Never compare its 8-second traveled distance with a real 0-mph reference.
  const feet70 = mph70.stopped ? mph70.distanceM * M_TO_FT : Number.NaN;
  const feet100 = mph100.stopped ? mph100.distanceM * M_TO_FT : Number.NaN;
  const ref70 = statusFor('braking70To0MphFt', feet70);
  const ref100 = statusFor('braking100To0MphFt', feet100);
  const referenceStatus = combineStatuses([ref70.status, ref100.status]);
  const status = allStopped ? referenceStatus : 'FAIL';

  const telemetryFile = writeTelemetry(artifactDir, 'braking', kmh100.rows);
  const graph = `${artifactDir}/braking-100kmh.svg`;
  writeLineChartSvg(graph, {
    title: '100–0 km/h braking — warmed driveline start',
    subtitle: 'A valid stopping distance exists only if the vehicle reaches ≤1 km/h',
    xLabel: 'time (s)',
    yLabel: 'scaled value',
    x: kmh100.rows.map((row) => Number(row.time_s)),
    series: [
      { name: 'speed km/h', values: kmh100.rows.map((row) => Number(row.speed_kmh)) },
      { name: 'decel g × 50', values: kmh100.rows.map((row) => -Number(row.longitudinal_g) * 50) },
    ],
  });

  const summary = allStopped
    ? `100–0 km/h ${kmh100.distanceM.toFixed(2)} m; 70–0 mph ${(feet70).toFixed(1)} ft; 100–0 mph ${(feet100).toFixed(1)} ft.`
    : `FAIL: full brake did not bring the car to rest within 8 s; 100 km/h run ended at ${kmh100.finalSpeedKmh.toFixed(2)} km/h.`;

  return {
    id: 'braking',
    name: 'Braking validation: 100–0 km/h, 70–0 mph and 100–0 mph',
    status,
    validationClass: 'hard',
    blocking: !allStopped,
    summary,
    metrics: {
      braking100To0KmhM: kmh100.stopped ? kmh100.distanceM : null,
      braking100To0KmhSec: kmh100.stopped ? kmh100.elapsedSec : null,
      braking100To0KmhActualStartKmh: kmh100.reachedKmh,
      braking100To0KmhStopped: kmh100.stopped ? 1 : 0,
      braking100To0KmhFinalSpeedKmh: kmh100.finalSpeedKmh,
      braking100To0KmhMinimumSpeedKmh: kmh100.minSpeedKmh,
      braking100To0KmhPeakDecelG: kmh100.peakDecelG,
      braking100To0KmhAverageDecelMs2: kmh100.averageDecelMs2,
      braking70To0MphFt: mph70.stopped ? feet70 : null,
      braking70To0MphStopped: mph70.stopped ? 1 : 0,
      braking70To0MphFinalSpeedKmh: mph70.finalSpeedKmh,
      braking100To0MphFt: mph100.stopped ? feet100 : null,
      braking100To0MphStopped: mph100.stopped ? 1 : 0,
      braking100To0MphFinalSpeedKmh: mph100.finalSpeedKmh,
      absActiveFraction100Kmh: kmh100.absFraction,
      lowSpeedPositiveAccelerationFrames: kmh100.positiveAccelFramesBelow15Kmh,
      lowSpeedRearPositiveFxFrames: kmh100.rearPositiveFxFramesBelow15Kmh,
      peakBrakePitchDeg: kmh100.peakPitchDeg,
      frontLoadPeakN: kmh100.peakFrontLoadN,
      rearLoadMinimumN: kmh100.minRearLoadN,
      braking70ErrorPercent: mph70.stopped ? (ref70.errorPercent ?? null) : null,
      braking100MphErrorPercent: mph100.stopped ? (ref100.errorPercent ?? null) : null,
    },
    diagnostics: !allStopped ? [
      'PHYSICS INVARIANT FAILURE: full brake does not bring the vehicle to rest. The current runs settle into a low-speed limit cycle instead of reaching zero.',
      'Telemetry shows ABS continuing to cycle near the low-speed plateau while the driven rear tires intermittently produce positive longitudinal force under full brake.',
      'Most likely causal chain to investigate first: first-gear automatic torque-converter creep/driveline torque remains active under brake, ABS pressure modulation releases enough tire braking force for that positive rear Fx to re-accelerate the car, and the vehicle never reaches the ABS low-speed cutoff.',
      'Inspect brake-aware torque-converter creep logic, ABS low-speed disable/hysteresis, pressure reapply behavior, and rear wheel slip/torque interaction. Do not fix this with a stopping-distance multiplier.',
      'Any 70–0 or 100–0 mph distance from an incomplete run is intentionally withheld rather than falsely compared with Car and Driver.',
    ] : referenceStatus === 'FAIL' ? [
      'The car now stops, but the completed stopping distances remain outside the external references. Investigate brake torque delivery, ABS regulation, tire longitudinal force/slip and surface comparability before calibration changes.',
    ] : ['100–0 km/h remains descriptive until a directly comparable external G90 reference is found.'],
    reference: ref70.reference,
    telemetryFile,
    graphFiles: [graph],
  };
}
