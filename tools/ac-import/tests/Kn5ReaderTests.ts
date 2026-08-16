import { parseKn5 } from '../Kn5Reader';
import { parseSurfacesIni } from '../SurfacesIni';

function assert(condition: unknown, message: string): asserts condition { if (!condition) throw new Error(message); }
function int32(value: number) { const b = Buffer.alloc(4); b.writeInt32LE(value); return b; }
function int16(value: number) { const b = Buffer.alloc(2); b.writeInt16LE(value); return b; }
function uint16(value: number) { const b = Buffer.alloc(2); b.writeUInt16LE(value); return b; }
function f32(value: number) { const b = Buffer.alloc(4); b.writeFloatLE(value); return b; }
function str32(value: string) { const bytes = Buffer.from(value, 'utf8'); return Buffer.concat([int32(bytes.length), bytes]); }
function vertex(x: number, y: number, z: number, u: number, v: number) {
  return Buffer.concat([f32(x), f32(y), f32(z), f32(0), f32(1), f32(0), f32(u), f32(v), Buffer.alloc(12)]);
}

const fixture = Buffer.concat([
  Buffer.from('sc6969'), int32(5), int32(0), int32(1),
  str32('roadmat'), str32('ksPerPixel'), int16(0), int32(0), int32(0), int32(0),
  int32(2), str32('1ROAD_TEST'), int32(0), Buffer.from([1, 0, 0, 0]), int32(3),
  vertex(0, 0, 0, 0, 0), vertex(2, 0, 0, 1, 0), vertex(0, 1, 2, 0, 1),
  int32(3), uint16(0), uint16(1), uint16(2), int32(0), Buffer.alloc(29),
]);

const model = parseKn5(fixture);
assert(model.version === 5, 'KN5 version');
assert(model.materials.length === 1, 'material count');
assert(model.meshes.length === 1, 'mesh count');
assert(model.meshes[0].name === '1ROAD_TEST', 'mesh name');
assert(model.meshes[0].positions.length === 9, 'vertex positions');
assert(model.meshes[0].indices[2] === 2, 'indices');
assert(Math.abs(model.meshes[0].uvs[1] - 1) < 1e-6, 'V coordinate should be flipped for Three.js');

const surfaces = parseSurfacesIni(`
[SURFACE_0]
KEY=ROAD
FRICTION=0.98
DAMPING=0
IS_VALID_TRACK=1

[SURFACE_1]
KEY=GRASS
FRICTION=0.55
IS_VALID_TRACK=0
`);
assert(surfaces.length === 2, 'surface count');
assert(surfaces[0].key === 'ROAD', 'surface key');
assert(Math.abs(surfaces[0].friction - 0.98) < 1e-6, 'surface friction');
assert(surfaces[1].isValidTrack === false, 'invalid-track flag should be preserved, not treated as no collision');
console.log('KN5 importer tests: PASS');
