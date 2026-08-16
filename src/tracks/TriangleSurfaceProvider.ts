import { ISurfaceProvider, SurfaceSample } from '../physics/SurfaceProvider';
import { PhysicsMath, Vec3 } from '../physics/math/PhysicsMath';
import { TrackSurfaceMaterial } from './TrackTypes';

export interface CollisionTriangle {
  ax: number;
  ay: number;
  az: number;
  bx: number;
  by: number;
  bz: number;
  cx: number;
  cy: number;
  cz: number;
  surfaceId: number;
}

interface IndexedTriangle extends CollisionTriangle {
  normal: Vec3;
  minX: number;
  maxX: number;
  minZ: number;
  maxZ: number;
}

const AIRBORNE_FALLBACK_Y = -10000;
const BARY_EPS = 1e-5;

/**
 * Exact triangle sampler for imported road/terrain meshes.
 * The grid accelerates candidate lookup; elevation and normal still come from
 * the source triangle. referenceY disambiguates stacked road layers.
 */
export class TriangleSurfaceProvider implements ISurfaceProvider {
  private readonly triangles: IndexedTriangle[];
  private readonly surfacesById: Map<number, TrackSurfaceMaterial>;
  private readonly cells = new Map<string, number[]>();

  constructor(
    triangles: CollisionTriangle[],
    surfaces: TrackSurfaceMaterial[],
    private readonly cellSize: number = 12
  ) {
    this.surfacesById = new Map(surfaces.map((surface) => [surface.id, surface]));
    this.triangles = triangles.map((triangle) => this.prepareTriangle(triangle));
    this.buildSpatialIndex();
  }

  public sampleSurface(x: number, z: number, referenceY?: number): SurfaceSample {
    const candidates = this.cells.get(this.cellKey(x, z));
    if (!candidates || candidates.length === 0) return this.airborneFallback();

    let best: { triangle: IndexedTriangle; y: number; verticalDistance: number } | null = null;

    for (const triangleIndex of candidates) {
      const triangle = this.triangles[triangleIndex];
      if (x < triangle.minX - BARY_EPS || x > triangle.maxX + BARY_EPS || z < triangle.minZ - BARY_EPS || z > triangle.maxZ + BARY_EPS) continue;

      const bary = this.barycentricXZ(x, z, triangle);
      if (!bary) continue;

      const y = bary.u * triangle.ay + bary.v * triangle.by + bary.w * triangle.cy;
      const verticalDistance = referenceY === undefined
        ? -y
        : referenceY >= y - 0.25
          ? referenceY - y
          : Number.POSITIVE_INFINITY;

      if (!Number.isFinite(verticalDistance)) continue;
      if (!best) best = { triangle, y, verticalDistance };
      else if (referenceY === undefined ? y > best.y : verticalDistance < best.verticalDistance) best = { triangle, y, verticalDistance };
    }

    if (!best) return this.airborneFallback();

    const material = this.surfacesById.get(best.triangle.surfaceId) ?? this.defaultSurface();
    const normal = best.triangle.normal.y < 0 ? PhysicsMath.vec3Scale(best.triangle.normal, -1) : best.triangle.normal;
    const slopePitch = Math.atan2(-normal.z, Math.max(1e-6, normal.y));
    const slopeRoll = Math.atan2(-normal.x, Math.max(1e-6, normal.y));

    return {
      elevation: best.y,
      normal,
      slopePitch,
      slopeRoll,
      type: material.type as any,
      friction: material.friction,
      rollingResistance: material.rollingResistance,
      wetness: material.wetness,
      isKerbRumble: material.isKerbRumble,
    };
  }

  private prepareTriangle(triangle: CollisionTriangle): IndexedTriangle {
    const ab = PhysicsMath.vec3(triangle.bx - triangle.ax, triangle.by - triangle.ay, triangle.bz - triangle.az);
    const ac = PhysicsMath.vec3(triangle.cx - triangle.ax, triangle.cy - triangle.ay, triangle.cz - triangle.az);
    let normal = PhysicsMath.vec3Normalize(PhysicsMath.vec3Cross(ab, ac));
    if (normal.y < 0) normal = PhysicsMath.vec3Scale(normal, -1);

    return {
      ...triangle,
      normal,
      minX: Math.min(triangle.ax, triangle.bx, triangle.cx),
      maxX: Math.max(triangle.ax, triangle.bx, triangle.cx),
      minZ: Math.min(triangle.az, triangle.bz, triangle.cz),
      maxZ: Math.max(triangle.az, triangle.bz, triangle.cz),
    };
  }

  private buildSpatialIndex() {
    for (let i = 0; i < this.triangles.length; i++) {
      const triangle = this.triangles[i];
      const minCellX = Math.floor(triangle.minX / this.cellSize);
      const maxCellX = Math.floor(triangle.maxX / this.cellSize);
      const minCellZ = Math.floor(triangle.minZ / this.cellSize);
      const maxCellZ = Math.floor(triangle.maxZ / this.cellSize);

      for (let cellX = minCellX; cellX <= maxCellX; cellX++) {
        for (let cellZ = minCellZ; cellZ <= maxCellZ; cellZ++) {
          const key = `${cellX}:${cellZ}`;
          const list = this.cells.get(key);
          if (list) list.push(i);
          else this.cells.set(key, [i]);
        }
      }
    }
  }

  private cellKey(x: number, z: number): string {
    return `${Math.floor(x / this.cellSize)}:${Math.floor(z / this.cellSize)}`;
  }

  private barycentricXZ(x: number, z: number, triangle: IndexedTriangle): { u: number; v: number; w: number } | null {
    const v0x = triangle.bx - triangle.ax;
    const v0z = triangle.bz - triangle.az;
    const v1x = triangle.cx - triangle.ax;
    const v1z = triangle.cz - triangle.az;
    const v2x = x - triangle.ax;
    const v2z = z - triangle.az;

    const denom = v0x * v1z - v1x * v0z;
    if (Math.abs(denom) < 1e-10) return null;

    const v = (v2x * v1z - v1x * v2z) / denom;
    const w = (v0x * v2z - v2x * v0z) / denom;
    const u = 1 - v - w;

    if (u < -BARY_EPS || v < -BARY_EPS || w < -BARY_EPS) return null;
    return { u, v, w };
  }

  private airborneFallback(): SurfaceSample {
    const surface = this.defaultSurface();
    return {
      elevation: AIRBORNE_FALLBACK_Y,
      normal: PhysicsMath.vec3(0, 1, 0),
      slopePitch: 0,
      slopeRoll: 0,
      type: surface.type as any,
      friction: surface.friction,
      rollingResistance: surface.rollingResistance,
      wetness: surface.wetness,
      isKerbRumble: false,
    };
  }

  private defaultSurface(): TrackSurfaceMaterial {
    return {
      id: -1,
      key: 'UNKNOWN',
      name: 'Unknown',
      type: 'unknown',
      friction: 0.5,
      rollingResistance: 0.08,
      wetness: 0,
      isKerbRumble: false,
    };
  }
}

export function decodeCollisionTriangles(buffer: ArrayBuffer, triangleCount: number): CollisionTriangle[] {
  const recordBytes = 40;
  if (buffer.byteLength < triangleCount * recordBytes) throw new Error(`Collision buffer is truncated: expected at least ${triangleCount * recordBytes} bytes, got ${buffer.byteLength}.`);

  const view = new DataView(buffer);
  const triangles: CollisionTriangle[] = new Array(triangleCount);
  let offset = 0;

  for (let i = 0; i < triangleCount; i++) {
    triangles[i] = {
      ax: view.getFloat32(offset, true), ay: view.getFloat32(offset + 4, true), az: view.getFloat32(offset + 8, true),
      bx: view.getFloat32(offset + 12, true), by: view.getFloat32(offset + 16, true), bz: view.getFloat32(offset + 20, true),
      cx: view.getFloat32(offset + 24, true), cy: view.getFloat32(offset + 28, true), cz: view.getFloat32(offset + 32, true),
      surfaceId: view.getUint32(offset + 36, true),
    };
    offset += recordBytes;
  }

  return triangles;
}
