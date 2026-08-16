import { access } from 'node:fs/promises';
import { basename, dirname, join, resolve } from 'node:path';
import { readKn5File } from './Kn5Reader';
import { readSurfacesIni } from './SurfacesIni';
import { defaultOutputDirectory, defaultTrackId, writeTrackPackage } from './TrackPackageWriter';

interface CliOptions {
  kn5Path: string;
  surfacesPath?: string;
  outputDir?: string;
  id?: string;
  name?: string;
  spawnX?: number;
  spawnZ?: number;
  spawnYaw?: number;
}

function usage(): never {
  console.error(`Usage:\n  npm run import:ac-track -- /path/to/track.kn5 [--surfaces /path/to/surfaces.ini] [--out ./public/tracks/id] [--id id] [--name "Track Name"] [--spawn-x 0 --spawn-z 0 --spawn-yaw 0]\n\nThe importer runs offline. The browser never parses KN5 files.`);
  process.exit(1);
}

function parseArgs(argv: string[]): CliOptions {
  if (argv.length === 0 || argv.includes('--help') || argv.includes('-h')) usage();
  const options: CliOptions = { kn5Path: resolve(argv[0]) };
  for (let i = 1; i < argv.length; i++) {
    const flag = argv[i], value = argv[++i];
    if (value === undefined) usage();
    switch (flag) {
      case '--surfaces': options.surfacesPath = resolve(value); break;
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

async function main() {
  const options = parseArgs(process.argv.slice(2));
  const id = options.id ?? defaultTrackId(options.kn5Path);
  const name = options.name ?? basename(options.kn5Path).replace(/\.kn5$/i, '');
  const candidateSurfaces = options.surfacesPath ?? join(dirname(options.kn5Path), 'data', 'surfaces.ini');
  const surfacesPath = await exists(candidateSurfaces) ? candidateSurfaces : undefined;
  const surfaces = surfacesPath ? await readSurfacesIni(surfacesPath) : [];

  console.log(`Reading KN5: ${options.kn5Path}`);
  const model = await readKn5File(options.kn5Path);
  console.log(`KN5 v${model.version}: ${model.meshes.length} meshes, ${model.materials.length} materials, ${model.textures.length} embedded textures.`);
  if (surfacesPath) console.log(`Surface definitions: ${surfacesPath} (${surfaces.length})`);
  else console.warn('No surfaces.ini found; collision extraction will use conservative mesh-name heuristics.');

  const outputDir = options.outputDir ?? defaultOutputDirectory(options.kn5Path, id);
  const manifest = await writeTrackPackage(model, surfaces, {
    id, name, modelFilename: basename(options.kn5Path), outputDir,
    spawn: {
      x: Number.isFinite(options.spawnX) ? options.spawnX! : 0,
      z: Number.isFinite(options.spawnZ) ? options.spawnZ! : 0,
      yaw: Number.isFinite(options.spawnYaw) ? options.spawnYaw! : 0,
    },
  });

  console.log(`Imported ${manifest.collisionTriangleCount.toLocaleString()} physical surface triangles.`);
  console.log(`Track package: ${join(outputDir, 'track.json')}`);
}

main().catch((error) => { console.error(error instanceof Error ? error.stack ?? error.message : error); process.exitCode = 1; });
