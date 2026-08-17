import { Simulation } from '../Simulation';
import { ValidationSurfaceProvider } from './ValidationSurfaceProvider';
import { writeLineChartSvg } from './ValidationArtifacts';
import {
  CONFIG, DT, NEUTRAL, basicRow, dominantFrequency, mean, maxAbs, setSpeed,
  writeTelemetry, type CorrectedValidationResult,
} from './CorrectedValidationCommon';

function runBump(kind: 'bump-left' | 'bump-full', axle: 'front' | 'rear') {
  const surface = new ValidationSurfaceProvider({
    kind,
    bumpStartZ: 20,
    bumpLengthM: 0.55,
    bumpHeightM: 0.025,
    friction: 1.0,
  });
  const sim = new Simulation(CONFIG, surface);
  const props = sim.vehicle.chassisMassProperties;
  const startZ = axle === 'front'
    ? 20 - props.cgToFrontAxle - 1.0
    : 20 + props.cgToRearAxle - 0.9;
  sim.reset(0, startZ, 0);
  for (let i = 0; i < Math.round(2.5 / DT); i++) sim.stepExplicit(NEUTRAL as any, 1);
  setSpeed(sim, 30 / 3.6);

  const wheelIndex = axle === 'front' ? 0 : 2;
  const prefix = wheelIndex === 0 ? 'fl' : 'rl';
  const rows: Record<string, unknown>[] = [];
  for (let i = 0; i < Math.round(2.5 / DT); i++) {
    const controls = { ...NEUTRAL, throttle: 0.08 };
    sim.stepExplicit(controls as any, 1);
    const row = basicRow(sim, i * DT, controls);
    const contact = sim.vehicle.suspension.states[wheelIndex].contactPointWorld;
    row.road_elevation_m = surface.sampleSurface(contact.x, contact.z).elevation;
    rows.push(row);
  }

  const roadOnsetIndex = rows.findIndex((row) => Number(row.road_elevation_m) > 0.0002);
  const onset = Math.max(0, roadOnsetIndex);
  const preStart = Math.max(0, onset - 12);
  const baselineRows = rows.slice(preStart, Math.max(preStart + 1, onset));
  const baseHubY = mean(baselineRows.map((row) => Number(row[`${prefix}_hub_world_y_m`])));
  const baseBodyY = mean(baselineRows.map((row) => Number(row.y_m)));
  const hubDelta = rows.map((row) => Number(row[`${prefix}_hub_world_y_m`]) - baseHubY);
  const bodyDelta = rows.map((row) => Number(row.y_m) - baseBodyY);

  const responseTime = (signal: number[], threshold: number) => {
    for (let i = onset; i < signal.length; i++) {
      if (Math.abs(signal[i]) >= threshold) return Number(rows[i].time_s);
    }
    return null;
  };
  const roadOnsetSec = Number(rows[onset]?.time_s ?? 0);
  const hubResponseSec = responseTime(hubDelta, 0.0005);
  const bodyResponseSec = responseTime(bodyDelta, 0.0003);

  const roadEndIndex = rows.findIndex((row, i) => i > onset && Number(row.road_elevation_m) <= 0.0002);
  const spectralStart = Math.min(
    Math.max(0, rows.length - 60),
    Math.max(onset + 1, (roadEndIndex > 0 ? roadEndIndex : onset) + Math.round(0.04 / DT))
  );

  return {
    rows,
    roadOnsetSec,
    hubResponseSec,
    bodyResponseSec,
    hubDelaySec: hubResponseSec === null ? null : hubResponseSec - roadOnsetSec,
    bodyDelaySec: bodyResponseSec === null ? null : bodyResponseSec - roadOnsetSec,
    wheelHopHz: dominantFrequency(hubDelta.slice(spectralStart), 5, 25),
    bodyHeaveHz: dominantFrequency(bodyDelta.slice(spectralStart), 0.5, 5),
    hubPeakM: maxAbs(hubDelta),
    bodyPeakM: maxAbs(bodyDelta),
    hubDelta,
    bodyDelta,
  };
}

export function runBumpValidation(artifactDir: string): CorrectedValidationResult {
  const front = runBump('bump-left', 'front');
  const rear = runBump('bump-left', 'rear');
  const full = runBump('bump-full', 'front');
  const responsesExist = [front, rear].every((run) =>
    run.hubResponseSec !== null && run.bodyResponseSec !== null
  );
  const temporalSeparationResolved = responsesExist && [front, rear].every((run) =>
    (run.bodyResponseSec as number) - (run.hubResponseSec as number) >= 0.5 * DT
  );

  const telemetryFile = writeTelemetry(artifactDir, 'bump-response', front.rows);
  const graph = `${artifactDir}/bump-response.svg`;
  writeLineChartSvg(graph, {
    title: 'Single-front-wheel bump — unsprung hub vs chassis',
    subtitle: 'Timing begins when the contact patch reaches the road bump',
    xLabel: 'time (s)',
    yLabel: 'vertical displacement from pre-bump baseline (m)',
    x: front.rows.map((row) => Number(row.time_s)),
    series: [
      { name: 'FL hub', values: front.hubDelta },
      { name: 'chassis CG', values: front.bodyDelta },
    ],
    markerX: front.roadOnsetSec,
    markerLabel: 'road contact',
  });

  const status = !responsesExist ? 'FAIL' : temporalSeparationResolved ? 'NO REFERENCE DATA' : 'WARNING';
  return {
    id: 'bump-response',
    name: 'Single-wheel/full-width bump and wheel-hop response',
    status,
    validationClass: 'engineering-plausibility',
    blocking: !responsesExist,
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
      wheelBeforeBodyTemporalSeparationResolved: temporalSeparationResolved ? 1 : 0,
    },
    diagnostics: [
      ...(!responsesExist ? ['The bump test failed to produce measurable wheel/chassis response. Inspect contact and suspension state before interpreting frequencies.'] : []),
      ...(responsesExist && !temporalSeparationResolved ? [
        'WARNING: hub and chassis displacement cross the current response thresholds within the same 120 Hz sample. The suite cannot yet quantitatively defend the required wheel-first/chassis-second delay.',
        'The branch is stacked on PR #27, while PR #22 (routing suspension chassis-force reaction instead of tire normal load directly into the rigid body) remains separate. Re-run this test after that force-path work is integrated.',
        'Raw rigid-body acceleration, tire-normal force and suspension chassis-force telemetry are now recorded so the next comparison can resolve this causal path directly.',
      ] : []),
      'REFERENCE DATA NEEDED for production G90 wheel-hop, body-heave frequency and damping ratio.',
    ],
    telemetryFile,
    graphFiles: [graph],
  };
}
