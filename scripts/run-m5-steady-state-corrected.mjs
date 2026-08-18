import { readFileSync, writeFileSync, unlinkSync } from 'node:fs';
import { spawnSync } from 'node:child_process';

const sourcePath = 'src/physics/validation/M5ValidationSuite.ts';
const tempPath = 'src/physics/validation/.M5ValidationSuite.steady-state.ts';
let source = readFileSync(sourcePath, 'utf8');

// The threshold helper previously missed a response that was already above the
// threshold in the first 120 Hz sample. That made step-steer chronology fail
// even when steering physically preceded yaw buildup.
source = source.replace(
  "  for (let i = Math.max(1, startIndex); i < samples.length; i++) {",
  "  if (samples.length && startIndex === 0 && getter(samples[0]) >= threshold) return samples[0].t;\n  for (let i = Math.max(1, startIndex); i < samples.length; i++) {"
);

// The skidpad driver was left in first gear. At the C/D 45.72 m radius it hit
// the rev limiter at ~57 km/h, so requested 0.6–1.05 g points all collapsed to
// ~0.56 g. Use the same normal powertrain shift path already used by the
// acceleration validation; no chassis velocity/pose/force is injected.
source = source.replace(
  "      this.step(driver(t, this));\n",
  "      this.step(driver(t, this));\n      autoShiftIfNeeded(this);\n"
);

writeFileSync(tempPath, source);
const args = [
  'tsx', tempPath,
  '--test=skidpad,step-steer',
  '--artifacts=artifacts/m5-validation/steady-state-corrected',
];
const result = spawnSync('npx', args, { stdio: 'inherit', shell: process.platform === 'win32' });
try { unlinkSync(tempPath); } catch {}
process.exit(result.status ?? 1);
