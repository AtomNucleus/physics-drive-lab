import { existsSync, readFileSync } from 'node:fs';
import { M5_REFERENCE_DATA, M5_REFERENCE_DATA_NEEDED } from './M5ReferenceData';
import { runCorrectedValidationTests } from './M5ValidationCorrectedTests';
import { ensureArtifactDir, writeJson, writeMarkdown, writeRowsCsv } from './ValidationArtifacts';

const DT = 1 / 120;

function parseArg(name: string): string | null {
  const exact = process.argv.find((arg) => arg.startsWith(`--${name}=`));
  if (exact) return exact.slice(name.length + 3);
  const idx = process.argv.indexOf(`--${name}`);
  if (idx >= 0 && process.argv[idx + 1] && !process.argv[idx + 1].startsWith('--')) return process.argv[idx + 1];
  return null;
}

function metricIndex(report: any): Record<string, number> {
  const out: Record<string, number> = {};
  for (const result of report.results ?? []) {
    for (const [metric, value] of Object.entries(result.metrics ?? {})) {
      if (typeof value === 'number' && Number.isFinite(value)) out[`${result.id}.${metric}`] = value;
    }
  }
  return out;
}

function regressionDeltas(current: any, baseline: any) {
  const a = metricIndex(baseline);
  const b = metricIndex(current);
  return Object.keys(b)
    .filter((metric) => Number.isFinite(a[metric]))
    .map((metric) => ({
      metric,
      before: a[metric],
      after: b[metric],
      percent: Math.abs(a[metric]) > 1e-12 ? ((b[metric] - a[metric]) / Math.abs(a[metric])) * 100 : null,
    }))
    .sort((x, y) => Math.abs((y.percent as number) ?? 0) - Math.abs((x.percent as number) ?? 0));
}

function main() {
  const artifactDir = parseArg('artifacts') ?? 'artifacts/m5-validation';
  const baseDir = parseArg('base') ?? `${artifactDir}/base`;
  const baseReportPath = `${baseDir}/m5-validation-report.json`;
  if (!existsSync(baseReportPath)) {
    throw new Error(`Base validation report not found: ${baseReportPath}. Run the core subset first.`);
  }
  ensureArtifactDir(artifactDir);
  const base = JSON.parse(readFileSync(baseReportPath, 'utf8'));
  const corrected = runCorrectedValidationTests(artifactDir);
  const replacement = new Map(corrected.map((result) => [result.id, result]));
  const results = (base.results ?? []).map((result: any) => replacement.get(result.id) ?? result);
  for (const result of corrected) {
    if (!results.some((existing: any) => existing.id === result.id)) results.push(result);
  }

  const statusCounts = results.reduce((acc: Record<string, number>, result: any) => {
    acc[result.status] = (acc[result.status] ?? 0) + 1;
    return acc;
  }, {});

  const report: any = {
    schemaVersion: 2,
    generatedAt: new Date().toISOString(),
    vehicleConfiguration: '2025 BMW M5 G90 validation calibration',
    fixedDtSec: DT,
    fixedPhysicsHz: 1 / DT,
    coordinateContract: '+X left, +Y up, +Z forward; positive steer/yaw = left; wheel order FL/FR/RL/RR',
    antiGamingRule: 'All measurements use the normal Simulation/Vehicle path; validation code prescribes only driver inputs, road geometry/material and initial conditions.',
    harnessRevision: 'v2 hardened: normal-driveline brake entry, validated skidpad speed/radius hold, first-sample step thresholds, actual unsprung hub bump telemetry, per-step energy accounting',
    statusCounts,
    results,
    references: Object.values(M5_REFERENCE_DATA),
    referenceDataNeeded: M5_REFERENCE_DATA_NEEDED,
    placeholders: base.placeholders ?? {},
  };

  const baselinePath = parseArg('baseline');
  if (baselinePath && existsSync(baselinePath)) {
    report.regressionDeltas = regressionDeltas(report, JSON.parse(readFileSync(baselinePath, 'utf8')));
  }

  writeJson(`${artifactDir}/m5-validation-report.json`, report);
  writeMarkdown(`${artifactDir}/m5-validation-report.md`, report);
  writeRowsCsv(`${artifactDir}/m5-validation-metrics.csv`, results.flatMap((result: any) =>
    Object.entries(result.metrics ?? {}).map(([metric, value]) => ({
      test: result.id,
      status: result.status,
      validation_class: result.validationClass,
      metric,
      value,
    }))
  ));

  console.log('\n2025 BMW M5 Vehicle Dynamics Validation — hardened report');
  console.log(`PASS ${statusCounts.PASS ?? 0} | WARNING ${statusCounts.WARNING ?? 0} | FAIL ${statusCounts.FAIL ?? 0} | NO REFERENCE DATA ${statusCounts['NO REFERENCE DATA'] ?? 0}`);
  console.log(`Report: ${artifactDir}/m5-validation-report.json`);
  console.log(`Markdown: ${artifactDir}/m5-validation-report.md`);

  const blockingFailure = results.some((result: any) => result.blocking && result.status === 'FAIL');
  const strictFailure = process.argv.includes('--strict') && results.some((result: any) => result.status === 'FAIL');
  if (blockingFailure || strictFailure) process.exitCode = 1;
}

main();
