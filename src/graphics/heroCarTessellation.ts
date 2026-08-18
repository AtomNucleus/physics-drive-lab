import * as THREE from 'three';

export const HERO_CAR_TARGET_TRIANGLES = 320_000;
export const HERO_CAR_TRIANGLE_BUDGET = 480_000;

export interface HeroCarTessellationOptions {
  targetTriangles?: number;
  triangleBudget?: number;
  maxPassesPerMesh?: number;
  minTrianglesPerMesh?: number;
  maxNormalOffsetRatio?: number;
}

export interface HeroCarTessellationReport {
  sourceTriangles: number;
  outputTriangles: number;
  sourceVertices: number;
  outputVertices: number;
  tessellatedMeshes: number;
  tessellationPasses: number;
  triangleBudget: number;
  targetTriangles: number;
}

function countGeometryTriangles(geometry: THREE.BufferGeometry): number {
  const position = geometry.getAttribute('position');
  if (!position) return 0;
  return geometry.index
    ? Math.floor(geometry.index.count / 3)
    : Math.floor(position.count / 3);
}

function countGeometryVertices(geometry: THREE.BufferGeometry): number {
  return geometry.getAttribute('position')?.count ?? 0;
}

function countGroupGeometry(root: THREE.Object3D) {
  let triangles = 0;
  let vertices = 0;
  root.traverse((object) => {
    if (!(object instanceof THREE.Mesh)) return;
    triangles += countGeometryTriangles(object.geometry);
    vertices += countGeometryVertices(object.geometry);
  });
  return { triangles, vertices };
}

function curvedEdgeMidpoint(
  positions: THREE.BufferAttribute,
  normals: THREE.BufferAttribute,
  a: number,
  b: number,
  maxNormalOffsetRatio: number,
): [number, number, number] {
  const ax = positions.getX(a), ay = positions.getY(a), az = positions.getZ(a);
  const bx = positions.getX(b), by = positions.getY(b), bz = positions.getZ(b);
  const nxA = normals.getX(a), nyA = normals.getY(a), nzA = normals.getZ(a);
  const nxB = normals.getX(b), nyB = normals.getY(b), nzB = normals.getZ(b);

  const mx = (ax + bx) * 0.5;
  const my = (ay + by) * 0.5;
  const mz = (az + bz) * 0.5;

  // Project the linear midpoint onto each endpoint tangent plane and average the
  // results. This is a light-weight point/normal tessellation approximation: it
  // adds useful curvature to the compact hero-car mesh instead of merely
  // splitting triangles without changing the rendered surface.
  const dotA = (mx - ax) * nxA + (my - ay) * nyA + (mz - az) * nzA;
  const dotB = (mx - bx) * nxB + (my - by) * nyB + (mz - bz) * nzB;
  const qAx = mx - nxA * dotA;
  const qAy = my - nyA * dotA;
  const qAz = mz - nzA * dotA;
  const qBx = mx - nxB * dotB;
  const qBy = my - nyB * dotB;
  const qBz = mz - nzB * dotB;

  let ox = (qAx + qBx) * 0.5 - mx;
  let oy = (qAy + qBy) * 0.5 - my;
  let oz = (qAz + qBz) * 0.5 - mz;

  const edgeX = bx - ax, edgeY = by - ay, edgeZ = bz - az;
  const edgeLength = Math.hypot(edgeX, edgeY, edgeZ);
  const maxOffset = edgeLength * Math.max(0, maxNormalOffsetRatio);
  const offsetLength = Math.hypot(ox, oy, oz);
  if (offsetLength > maxOffset && offsetLength > 1e-9) {
    const scale = maxOffset / offsetLength;
    ox *= scale;
    oy *= scale;
    oz *= scale;
  }

  return [mx + ox, my + oy, mz + oz];
}

function subdivideIndexedGeometryOnce(
  source: THREE.BufferGeometry,
  maxNormalOffsetRatio: number,
): THREE.BufferGeometry | null {
  const position = source.getAttribute('position') as THREE.BufferAttribute | undefined;
  const index = source.index;
  if (!position || position.itemSize < 3 || !index || index.count < 3) return null;

  // The bundled M5 contains positions + indices only. Compute smooth normals for
  // the point/normal interpolation before generating the new edge vertices.
  source.computeVertexNormals();
  const normal = source.getAttribute('normal') as THREE.BufferAttribute | undefined;
  if (!normal) return null;

  const nextPositions: number[] = [];
  for (let i = 0; i < position.count; i += 1) {
    nextPositions.push(position.getX(i), position.getY(i), position.getZ(i));
  }

  const midpointCache = new Map<string, number>();
  const midpointIndex = (a: number, b: number) => {
    const lo = Math.min(a, b);
    const hi = Math.max(a, b);
    const key = `${lo}:${hi}`;
    const cached = midpointCache.get(key);
    if (cached !== undefined) return cached;

    const midpoint = curvedEdgeMidpoint(position, normal, lo, hi, maxNormalOffsetRatio);
    const result = nextPositions.length / 3;
    nextPositions.push(midpoint[0], midpoint[1], midpoint[2]);
    midpointCache.set(key, result);
    return result;
  };

  const nextIndices: number[] = [];
  for (let i = 0; i + 2 < index.count; i += 3) {
    const a = index.getX(i);
    const b = index.getX(i + 1);
    const c = index.getX(i + 2);
    const ab = midpointIndex(a, b);
    const bc = midpointIndex(b, c);
    const ca = midpointIndex(c, a);

    nextIndices.push(
      a, ab, ca,
      ab, b, bc,
      ca, bc, c,
      ab, bc, ca,
    );
  }

  const result = new THREE.BufferGeometry();
  result.setAttribute('position', new THREE.Float32BufferAttribute(nextPositions, 3));
  result.setIndex(nextIndices);
  result.computeVertexNormals();
  result.computeBoundingBox();
  result.computeBoundingSphere();
  return result;
}

/**
 * Adds render-only geometry to a single hero vehicle. The physics/collision
 * representation is deliberately untouched. We tessellate the largest useful
 * meshes first until the target is reached, and never exceed the hard triangle
 * budget so vertex processing remains predictable at high frame rates.
 */
export function enhanceHeroCarGeometry(
  root: THREE.Object3D,
  options: HeroCarTessellationOptions = {},
): HeroCarTessellationReport {
  const triangleBudget = Math.max(0, options.triangleBudget ?? HERO_CAR_TRIANGLE_BUDGET);
  const requestedTargetTriangles = Math.max(0, options.targetTriangles ?? HERO_CAR_TARGET_TRIANGLES);
  const targetTriangles = Math.min(requestedTargetTriangles, triangleBudget);
  const maxPassesPerMesh = Math.max(0, Math.floor(options.maxPassesPerMesh ?? 2));
  const minTrianglesPerMesh = Math.max(1, Math.floor(options.minTrianglesPerMesh ?? 24));
  const maxNormalOffsetRatio = Math.max(0, options.maxNormalOffsetRatio ?? 0.035);

  const meshes: THREE.Mesh[] = [];
  root.traverse((object) => {
    if (!(object instanceof THREE.Mesh)) return;
    if (!object.geometry.index || !object.geometry.getAttribute('position')) return;
    meshes.push(object);
  });

  const source = countGroupGeometry(root);
  let currentTriangles = source.triangles;
  let tessellationPasses = 0;
  const touchedMeshes = new Set<THREE.Mesh>();
  const passes = new Map<THREE.Mesh, number>();

  while (currentTriangles < targetTriangles && tessellationPasses < meshes.length * maxPassesPerMesh) {
    let changedThisRound = false;
    const candidates = [...meshes].sort(
      (a, b) => countGeometryTriangles(b.geometry) - countGeometryTriangles(a.geometry),
    );

    for (const mesh of candidates) {
      const pass = passes.get(mesh) ?? 0;
      if (pass >= maxPassesPerMesh) continue;

      const triangleCount = countGeometryTriangles(mesh.geometry);
      if (triangleCount < minTrianglesPerMesh) continue;

      // One midpoint subdivision pass turns each source triangle into four.
      const addedTriangles = triangleCount * 3;
      if (currentTriangles + addedTriangles > triangleBudget) continue;

      // Use progressively smaller curvature offsets on later passes so the first
      // pass improves the compact LOD silhouette while subsequent passes mainly
      // increase smoothness without over-rounding BMW body creases.
      const passOffsetRatio = maxNormalOffsetRatio / (pass + 1);
      const nextGeometry = subdivideIndexedGeometryOnce(mesh.geometry, passOffsetRatio);
      if (!nextGeometry) continue;

      const previousGeometry = mesh.geometry;
      mesh.geometry = nextGeometry;
      previousGeometry.dispose();

      currentTriangles += addedTriangles;
      passes.set(mesh, pass + 1);
      touchedMeshes.add(mesh);
      tessellationPasses += 1;
      changedThisRound = true;

      if (currentTriangles >= targetTriangles) break;
    }

    if (!changedThisRound) break;
  }

  const output = countGroupGeometry(root);
  return {
    sourceTriangles: source.triangles,
    outputTriangles: output.triangles,
    sourceVertices: source.vertices,
    outputVertices: output.vertices,
    tessellatedMeshes: touchedMeshes.size,
    tessellationPasses,
    triangleBudget,
    targetTriangles,
  };
}
