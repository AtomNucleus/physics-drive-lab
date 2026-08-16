import { access } from 'node:fs/promises';
import { basename, dirname, extname, join, resolve } from 'node:path';
import { mergeKn5Models, readKn5File } from './Kn5Reader';
import { hasExternalModelTransform, readModelsIni } from './ModelsIni';
import { readSurfacesIni } from './SurfacesIni';
import { defaultOutputDirectory, defaultTrackId, writeTrackPackage } from './TrackPackageWriter';

interface CliOptions {
  inputPath: string;
  surfacesPath?: string;
  collisionKn5Path?: string;
  outputDir?: string;
  id?: string;
  name?: string;
  spawnX?: number;
  spawnZ?: number;
  spawnYaw?: number;
}

function usage(): never {
  console.error(`Usage:\n  npm run import:ac-track -- /path/to/track.kn5 [options]\n  npm run import:ac-track -- /path/to/models_layout.ini [options]\n\nOptions:\n  --surfaces /path/to/surfaces.ini\n  --collision-kn5 /path/to/physics.kn5\n  --out ./public/tracks/id\n  --id id\n  --name "Track Name"\n  --spawn-x 0 --spawn-z 0 --spawn-yaw 0\n\nFor AC layout INIs, the importer reads every MODEL_n KN5, automatically separates a physics/collision KN5 when its filename contains "physics" or "collision", and uses AC_START_0 as the default spawn when available. The importer runs offline; the browser never parses KN5 files.`);
  process.exit(1);
}

function parseArgs(argv: string[]): CliOptions {
  if (argv.length === 0 || argv.includes('--help') || argv.includes('-h')) usage();
  const options: CliOptions = { inputPath: resolve(argv[0]) };
  for (let i = 1; i < argv.length; i++) {
    const flag = argv[i], value = argv[++i];
    if (value === undefined) usage();
    switch (flag) {
      case '--surfaces': options.surfacesPath = resolve(value); break;
      case '--collision-kn5': options.collisionKn5Path = resolve(value); break;
      case '--out': options.outputDir = resolve(value); break;
      case '--id': options.id = value; break;
      case '--name': options.name = value; break;
      case '--spawn-x': options.spawnX = Number(value); break;
      case '--spawn-z': options.spawnZ = Number(value); break;
      case '--spawn-yaw': options.spawnYaw = Number(value); break;
      default: throw new Error(`Unknown option ${flag}. Use --help for usage.`);
    }
  }
  return options;
}

async function exists(path: string): Promise<boolean> { try { await access(path); return true; } catch { return false; } }

async function findSurfacesPath(inputPath: string, explicit?: string): Promise<string | undefined> {
  if (explicit) return await exists(explicit) ? explicit : undefined;
  const inputDir = dirname(inputPath);
  const candidates: string[] = [];
  if (extname(inputPath).toLowerCase() === '.ini') {
    const layoutName = basename(inputPath, extname(inputPath)).replace(/^models_/i, '');
    candidates.push(join(inputDir, layoutName, 'data', 'surfaces.ini'));
  }
  candidates.push(join(inputDir, 'data', 'surfaces.ini'));
  for (const candidate of candidates) if (await exists(candidate)) return candidate;
  return undefined;
}

async function loadInput(options: CliOptions) {
  const isLayout = extname(options.inputPath).toLowerCase() === '.ini';

  if (!isLayout) {
    console.log(`Reading visual KN5: ${options.inputPath}`);
    const visualModel = await readKn5File(options.inputPath);
    let collisionModel = visualModel;
    let collisionFilename = basename(options.inputPath);
    if (options.collisionKn5Path && resolve(options.collisionKn5Path) !== resolve(options.inputPath)) {
      console.log(`Reading collision KN5: ${options.collisionKn5Path}`);
      collisionModel = await readKn5File(options.collisionKn5Path);
      collisionFilename = basename(options.collisionKn5Path);
    }
    return {
      visualModel,
      collisionModel,
      sourceModels: [basename(options.inputPath)],
      collisionFilename,
    };
  }

  const layoutModels = await readModelsIni(options.inputPath);
  if (layoutModels.length === 0) throw new Error(`No MODEL_n entries found in ${options.inputPath}.`);

  const transformed = layoutModels.filter(hasExternalModelTransform);
  if (transformed.length > 0) {
    throw new Error(`This layout uses non-zero MODEL_n POSITION/ROTATION transforms (${transformed.map((model) => model.file).join(', ')}). Explicit models.ini transforms are not supported yet; refusing to silently import incorrect geometry.`);
  }

  console.log(`Reading AC layout: ${options.inputPath} (${layoutModels.length} KN5 files)`);
  const loaded = [] as { file: string; path: string; model: Awaited<ReturnType<typeof readKn5File>> }[];
  for (const entry of layoutModels) {
    console.log(`  KN5: ${entry.file}`);
    loaded.push({ file: entry.file, path: entry.path, model: await readKn5File(entry.path) });
  }

  let collisionPath = options.collisionKn5Path;
  if (!collisionPath) {
    collisionPath = loaded.find((entry) => /physics|collision/i.test(entry.file))?.path;
  }

  let collisionFilename: string;
  let collisionModel;
  const collisionEntry = collisionPath ? loaded.find((entry) => resolve(entry.path) === resolve(collisionPath!)) : undefined;
  if (collisionEntry) {
    collisionModel = collisionEntry.model;
    collisionFilename = collisionEntry.file;
  } else if (collisionPath) {
    console.log(`  Collision KN5: ${collisionPath}`);
    collisionModel = await readKn5File(collisionPath);
    collisionFilename = basename(collisionPath);
  } else {
    collisionModel = mergeKn5Models(loaded.map((entry) => entry.model));
    collisionFilename = basename(options.inputPath);
    console.warn('No dedicated physics/collision KN5 detected; using the merged layout with mesh-name surface heuristics.');
  }

  const visualEntries = collisionEntry ? loaded.filter((entry) => entry !== collisionEntry) : loaded;
  const effectiveVisualEntries = visualEntries.length > 0 ? visualEntries : loaded;
  const visualModel = mergeKn5Models(effectiveVisualEntries.map((entry) => entry.model));

  return {
    visualModel,
    collisionModel,
    sourceModels: effectiveVisualEntries.map((entry) => entry.file),
    collisionFilename,
  };
}

async function main() {
  const options = parseArgs(process.argv.slice(2));
  const id = options.id ?? defaultTrackId(options.inputPath);
  const name = options.name ?? basename(options.inputPath, extname(options.inputPath)).replace(/^models_/i, '').replace(/[_-]+/g, ' ');
  const surfacesPath = await findSurfacesPath(options.inputPath, options.surfacesPath);
  const surfaces = surfacesPath ? await readSurfacesIni(surfacesPath) : [];

  const loaded = await loadInput(options);
  console.log(`Visual package source: ${loaded.visualModel.meshes.length} meshes, ${loaded.visualModel.materials.length} materials, ${loaded.visualModel.textures.length} unique embedded textures.`);
  console.log(`Collision source: ${loaded.collisionFilename} (${loaded.collisionModel.meshes.length} meshes).`);
  if (surfacesPath) console.log(`Surface definitions: ${surfacesPath} (${surfaces.length})`);
  else console.warn('No surfaces.ini found; collision extraction will use conservative mesh-name heuristics.');

  const autoSpawn = loaded.visualModel.markers.find((marker) => /^AC_START_0$/i.test(marker.name))
    ?? loaded.collisionModel.markers.find((marker) => /^AC_START_0$/i.test(marker.name));
  if (autoSpawn) console.log(`Auto spawn AC_START_0: x=${autoSpawn.x.toFixed(3)} z=${autoSpawn.z.toFixed(3)} yaw=${autoSpawn.yaw.toFixed(4)} rad.`);

  const outputDir = options.outputDir ?? defaultOutputDirectory(options.inputPath, id);
  const manifest = await writeTrackPackage(loaded.visualModel, surfaces, {
    id,
    name,
    modelFilename: basename(options.inputPath),
    sourceModels: loaded.sourceModels,
    collisionModelFilename: loaded.collisionFilename,
    outputDir,
    spawn: {
      x: Number.isFinite(options.spawnX) ? options.spawnX! : autoSpawn?.x ?? 0,
      z: Number.isFinite(options.spawnZ) ? options.spawnZ! : autoSpawn?.z ?? 0,
      yaw: Number.isFinite(options.spawnYaw) ? options.spawnYaw! : autoSpawn?.yaw ?? 0,
    },
  }, loaded.collisionModel);

  console.log(`Imported ${manifest.collisionTriangleCount.toLocaleString()} physical surface triangles.`);
  if (manifest.rejectedSteepTriangleCount > 0) console.log(`Rejected ${manifest.rejectedSteepTriangleCount.toLocaleString()} near-vertical/steep triangles from wheel-support collision.`);
  console.log(`Track package: ${join(outputDir, 'track.json')}`);
}

main().catch((error) => { console.error(error instanceof Error ? error.stack ?? error.message : error); process.exitCode = 1; });
