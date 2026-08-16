import { TriangleSurfaceProvider } from '../TriangleSurfaceProvider';
import { TrackSurfaceMaterial } from '../TrackTypes';

function assert(condition: unknown, message: string): asserts condition { if (!condition) throw new Error(message); }
function close(actual: number, expected: number, tolerance: number, message: string) {
  assert(Math.abs(actual - expected) <= tolerance, `${message}: expected ${expected}, got ${actual}`);
}

const surfaces: TrackSurfaceMaterial[] = [
  { id: 0, key: 'ROAD', name: 'Road', type: 'asphalt', friction: 1, rollingResistance: 0.015, wetness: 0, isKerbRumble: false },
  { id: 1, key: 'KERB', name: 'Kerb', type: 'kerb', friction: 0.88, rollingResistance: 0.024, wetness: 0, isKerbRumble: true },
];

const provider = new TriangleSurfaceProvider([
  { ax: -10, ay: -1, az: -10, bx: 10, by: -1, bz: -10, cx: 10, cy: 1, cz: 10, surfaceId: 0 },
  { ax: -10, ay: -1, az: -10, bx: 10, by: 1, bz: 10, cx: -10, cy: 1, cz: 10, surfaceId: 0 },
  { ax: -2, ay: 10, az: -2, bx: 2, by: 10, bz: -2, cx: 2, cy: 10, cz: 2, surfaceId: 1 },
  { ax: -2, ay: 10, az: -2, bx: 2, by: 10, bz: 2, cx: -2, cy: 10, cz: 2, surfaceId: 1 },
], surfaces, 5);

const slopeSample = provider.sampleSurface(5, 5, 2);
close(slopeSample.elevation, 0.5, 1e-5, 'slope elevation');
assert(slopeSample.normal.y > 0.99, 'road normal must point upward');
close(slopeSample.friction, 1, 1e-6, 'road friction');

const lowerLayer = provider.sampleSurface(0, 0, 2);
close(lowerLayer.elevation, 0, 1e-5, 'referenceY should choose lower road layer');
assert(lowerLayer.type === 'asphalt', 'lower layer surface classification');

const upperLayer = provider.sampleSurface(0, 0, 12);
close(upperLayer.elevation, 10, 1e-5, 'referenceY should choose bridge layer');
assert(upperLayer.type === 'kerb', 'upper layer surface classification');

const outside = provider.sampleSurface(200, 200, 2);
assert(outside.elevation < -1000, 'missing surface must not create an invisible floor');
console.log('TriangleSurfaceProvider tests: PASS');
