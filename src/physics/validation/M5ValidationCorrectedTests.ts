import { writeFileSync } from 'node:fs';
import { ensureArtifactDir } from './ValidationArtifacts';
import { runBrakingValidation } from './ValidationTestBraking';
import { runSkidpadValidation } from './ValidationTestSkidpad';
import { runStepSteerValidation } from './ValidationTestStepSteer';
import { runBumpValidation } from './ValidationTestBump';
import { runEnergyValidation } from './ValidationTestEnergy';
import { runSteeringChainValidation } from './ValidationTestSteeringChain';
import type { CorrectedValidationResult } from './CorrectedValidationCommon';

export function runCorrectedValidationTests(artifactDir: string): CorrectedValidationResult[] {
  ensureArtifactDir(artifactDir);
  const tests = [
    runBrakingValidation,
    runSkidpadValidation,
    runStepSteerValidation,
    runBumpValidation,
    runEnergyValidation,
  ];
  const results: CorrectedValidationResult[] = [];

  for (const test of tests) {
    const result = test(artifactDir);
    results.push(result);
    console.log(`[M5 validation hardened] ${result.id}: ${result.status} — ${result.summary}`);
    for (const diagnostic of result.diagnostics) console.log(`  - ${diagnostic}`);
  }

  const skidpad = results.find((result) => result.id === 'skidpad');
  const stepSteer = results.find((result) => result.id === 'step-steer');
  if (!skidpad || !stepSteer) throw new Error('Steering-chain validation requires hardened skidpad and step-steer results.');
  const steeringChain = runSteeringChainValidation(artifactDir, skidpad, stepSteer);
  results.push(steeringChain);
  console.log(`[M5 validation hardened] ${steeringChain.id}: ${steeringChain.status} — ${steeringChain.summary}`);
  for (const diagnostic of steeringChain.diagnostics) console.log(`  - ${diagnostic}`);

  writeFileSync(
    `${artifactDir}/corrected-results.json`,
    `${JSON.stringify(results, null, 2)}\n`,
    'utf8'
  );
  return results;
}
