import { mkdtemp, readFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { Kn5Model } from '../Kn5Reader';
import { parseSurfacesIni } from '../SurfacesIni';
import { writeTrackPackage } from '../TrackPackageWriter';

function assert(condition: unknown, message: string): asserts condition { if (!condition) throw new Error(message); }

const visualModel: Kn5Model = {
  version: 6,
  textures: [],
  materials: [],
  markers: [],
  meshes: [{
    name: 'SCENERY',
    materialId: 0,
    positions: new Float32Array([0, 0, 0, 1, 0, 0, 0, 0, 1]),
    normals: new Float32Array([0, 1, 0, 0, 1, 0, 0, 1, 0]),
    uvs: new Float32Array([0, 0, 1, 0, 0, 1]),
    indices: new Uint32Array([0, 1, 2]),
  }],
};

const collisionModel: Kn5Model = {
  version: 6,
  textures: [],
  materials: [],
  markers: [],
  meshes: [{
    name: '1ROAD_TEST',
    materialId: 0,
    positions: new Float32Array([
      0, 0, 0, 2, 0, 0, 0, 0, 2,
      10, 0, 0, 10, 2, 0, 10, 0, 2,
    ]),
    normals: new Float32Array(18),
    uvs: new Float32Array(12),
    indices: new Uint32Array([0, 1, 2, 3, 4, 5]),
  }],
};

const surfaces = parseSurfacesIni(`
[SURFACE_0]
KEY=ROAD
FRICTION=0.98
IS_VALID_TRACK=1
`);

const dir = await mkdtemp(join(tmpdir(), 'ac-track-writer-'));
try {
  const manifest = await writeTrackPackage(visualModel, surfaces, {
    id: 'fixture',
    name: 'Fixture',
    modelFilename: 'models_fixture.ini',
    sourceModels: ['visual.kn5'],
    collisionModelFilename: 'physics.kn5',
    outputDir: dir,
    spawn: { x: 12, z: 34, yaw: 1.5 },
  }, collisionModel);

  assert(manifest.collisionTriangleCount === 1, 'horizontal road triangle should be imported');
  assert(manifest.rejectedSteepTriangleCount === 1, 'vertical wall triangle should be rejected from wheel support');
  assert(manifest.source.collisionModel === 'physics.kn5', 'dedicated collision model should be recorded');
  assert(manifest.spawnPoints[0].x === 12 && manifest.spawnPoints[0].z === 34, 'spawn should be preserved');
  const collision = await readFile(join(dir, 'collision.bin'));
  assert(collision.byteLength === 40, 'one collision record should be written');
} finally {
  await rm(dir, { recursive: true, force: true });
}

console.log('AC track package writer tests: PASS');
