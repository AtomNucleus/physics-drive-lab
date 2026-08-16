import { mkdir, writeFile } from 'node:fs/promises';
import { basename, dirname, extname, join } from 'node:path';
import { Kn5Material, Kn5Model } from './Kn5Reader';
import { AcSurfaceDefinition } from './SurfacesIni';

type SurfaceKind = 'asphalt' | 'racing_line' | 'kerb' | 'wet' | 'marbles' | 'gravel' | 'grass' | 'sand' | 'dirt' | 'concrete' | 'unknown';

interface PackageSurface {
  id: number;
  key: string;
  name: string;
  type: SurfaceKind;
  friction: number;
  rollingResistance: number;
  wetness: number;
  isKerbRumble: boolean;
  isValidTrack: boolean;
}

interface WriterOptions {
  id: string;
  name: string;
  modelFilename: string;
  outputDir: string;
  sourceModels?: string[];
  collisionModelFilename?: string;
  spawn?: { x: number; z: number; yaw: number };
}

const align4 = (value: number) => (value + 3) & ~3;
const sanitizeFilename = (name: string) => name.replace(/[\\/:*?"<>|]/g, '_');
const MIN_DRIVEABLE_NORMAL_Y = 0.25;

function classifySurface(key: string): { type: SurfaceKind; rollingResistance: number; wetness: number; isKerbRumble: boolean } {
  const upper = key.toUpperCase();
  if (/KERB|CURB/.test(upper)) return { type: 'kerb', rollingResistance: 0.024, wetness: 0, isKerbRumble: true };
  if (/GRASS/.test(upper)) return { type: 'grass', rollingResistance: 0.08, wetness: 0, isKerbRumble: false };
  if (/GRAVEL/.test(upper)) return { type: 'gravel', rollingResistance: 0.075, wetness: 0, isKerbRumble: false };
  if (/SAND/.test(upper)) return { type: 'sand', rollingResistance: 0.11, wetness: 0, isKerbRumble: false };
  if (/DIRT|SOIL/.test(upper)) return { type: 'dirt', rollingResistance: 0.09, wetness: 0, isKerbRumble: false };
  if (/CONCRETE|CEMENT/.test(upper)) return { type: 'concrete', rollingResistance: 0.016, wetness: 0, isKerbRumble: false };
  if (/WET/.test(upper)) return { type: 'wet', rollingResistance: 0.015, wetness: 0.8, isKerbRumble: false };
  if (/RACING|LINE/.test(upper)) return { type: 'racing_line', rollingResistance: 0.015, wetness: 0, isKerbRumble: false };
  if (/ROAD|ASPHALT|TARMAC|PIT/.test(upper)) return { type: 'asphalt', rollingResistance: 0.015, wetness: 0, isKerbRumble: false };
  return { type: 'unknown', rollingResistance: 0.04, wetness: 0, isKerbRumble: false };
}

function surfaceForMesh(meshName: string, surfaces: AcSurfaceDefinition[]): AcSurfaceDefinition | null {
  const upper = meshName.toUpperCase();
  return surfaces.filter((surface) => upper.includes(surface.key)).sort((a, b) => b.key.length - a.key.length)[0] ?? null;
}

function fallbackSurfaceForMesh(meshName: string): AcSurfaceDefinition | null {
  const match = /(ROAD|ASPHALT|TARMAC|PIT|KERB|CURB|GRASS|GRAVEL|SAND|DIRT|SOIL|CONCRETE|CEMENT)/.exec(meshName.toUpperCase());
  if (!match) return null;
  return { section: 'HEURISTIC', key: match[1], friction: 1, damping: 0, isValidTrack: true, raw: {} };
}

function packageMaterial(material: Kn5Material) {
  return {
    id: material.id,
    name: material.name,
    shader: material.shader,
    diffuseTexture: material.textures.txDiffuse ? `textures/${sanitizeFilename(material.textures.txDiffuse)}` : undefined,
    normalTexture: material.textures.txNormal ? `textures/${sanitizeFilename(material.textures.txNormal)}` : undefined,
    diffuseMultiplier: material.properties.diffuseMult,
    specular: material.properties.ksSpecular,
    specularExponent: material.properties.ksSpecularEXP,
  };
}

function bufferFromTypedArray(array: Float32Array | Uint32Array): Buffer {
  return Buffer.from(array.buffer, array.byteOffset, array.byteLength);
}

function triangleAbsNormalY(positions: Float32Array, ia: number, ib: number, ic: number): number {
  const abx = positions[ib] - positions[ia];
  const aby = positions[ib + 1] - positions[ia + 1];
  const abz = positions[ib + 2] - positions[ia + 2];
  const acx = positions[ic] - positions[ia];
  const acy = positions[ic + 1] - positions[ia + 1];
  const acz = positions[ic + 2] - positions[ia + 2];
  const nx = aby * acz - abz * acy;
  const ny = abz * acx - abx * acz;
  const nz = abx * acy - aby * acx;
  const length = Math.hypot(nx, ny, nz);
  return length > 1e-8 ? Math.abs(ny / length) : 0;
}

export async function writeTrackPackage(
  visualModel: Kn5Model,
  surfacesIni: AcSurfaceDefinition[],
  options: WriterOptions,
  collisionModel: Kn5Model = visualModel
) {
  await mkdir(options.outputDir, { recursive: true });
  await mkdir(join(options.outputDir, 'textures'), { recursive: true });
  for (const texture of visualModel.textures) await writeFile(join(options.outputDir, 'textures', sanitizeFilename(texture.name)), texture.data);

  const visualChunks: Buffer[] = [];
  const meshManifest: any[] = [];
  let visualOffset = 0;
  const boundsMin = [Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY];
  const boundsMax = [Number.NEGATIVE_INFINITY, Number.NEGATIVE_INFINITY, Number.NEGATIVE_INFINITY];

  const append = (buffer: Buffer) => {
    const alignedOffset = align4(visualOffset);
    if (alignedOffset > visualOffset) visualChunks.push(Buffer.alloc(alignedOffset - visualOffset));
    visualOffset = alignedOffset;
    const offset = visualOffset;
    visualChunks.push(buffer);
    visualOffset += buffer.length;
    return offset;
  };

  for (const mesh of visualModel.meshes) {
    for (let i = 0; i < mesh.positions.length; i += 3) {
      boundsMin[0] = Math.min(boundsMin[0], mesh.positions[i]); boundsMin[1] = Math.min(boundsMin[1], mesh.positions[i + 1]); boundsMin[2] = Math.min(boundsMin[2], mesh.positions[i + 2]);
      boundsMax[0] = Math.max(boundsMax[0], mesh.positions[i]); boundsMax[1] = Math.max(boundsMax[1], mesh.positions[i + 1]); boundsMax[2] = Math.max(boundsMax[2], mesh.positions[i + 2]);
    }
    const positionsOffset = append(bufferFromTypedArray(mesh.positions));
    const normalsOffset = append(bufferFromTypedArray(mesh.normals));
    const uvsOffset = append(bufferFromTypedArray(mesh.uvs));
    const indicesOffset = append(bufferFromTypedArray(mesh.indices));
    meshManifest.push({
      name: mesh.name,
      materialId: mesh.materialId,
      positions: { byteOffset: positionsOffset, count: mesh.positions.length / 3 },
      normals: { byteOffset: normalsOffset, count: mesh.normals.length / 3 },
      uvs: { byteOffset: uvsOffset, count: mesh.uvs.length / 2 },
      indices: { byteOffset: indicesOffset, count: mesh.indices.length },
    });
  }
  await writeFile(join(options.outputDir, 'visual.bin'), Buffer.concat(visualChunks));

  const packageSurfaces: PackageSurface[] = [];
  const surfaceIds = new Map<string, number>();
  const collisionRecords: Buffer[] = [];
  let collisionTriangleCount = 0;
  let rejectedSteepTriangleCount = 0;

  const getPackageSurface = (source: AcSurfaceDefinition): PackageSurface => {
    const existingId = surfaceIds.get(source.key);
    if (existingId !== undefined) return packageSurfaces[existingId];
    const classification = classifySurface(source.key);
    const surface: PackageSurface = {
      id: packageSurfaces.length,
      key: source.key,
      name: source.section,
      type: classification.type,
      friction: Math.max(0.05, source.friction),
      rollingResistance: classification.rollingResistance,
      wetness: classification.wetness,
      isKerbRumble: classification.isKerbRumble,
      isValidTrack: source.isValidTrack,
    };
    packageSurfaces.push(surface); surfaceIds.set(source.key, surface.id); return surface;
  };

  for (const mesh of collisionModel.meshes) {
    const acSurface = surfaceForMesh(mesh.name, surfacesIni) ?? fallbackSurfaceForMesh(mesh.name);
    if (!acSurface) continue;
    const surface = getPackageSurface(acSurface);
    for (let i = 0; i + 2 < mesh.indices.length; i += 3) {
      const ia = mesh.indices[i] * 3, ib = mesh.indices[i + 1] * 3, ic = mesh.indices[i + 2] * 3;
      if (ic + 2 >= mesh.positions.length) continue;
      if (triangleAbsNormalY(mesh.positions, ia, ib, ic) < MIN_DRIVEABLE_NORMAL_Y) {
        rejectedSteepTriangleCount++;
        continue;
      }
      const record = Buffer.allocUnsafe(40);
      record.writeFloatLE(mesh.positions[ia], 0); record.writeFloatLE(mesh.positions[ia + 1], 4); record.writeFloatLE(mesh.positions[ia + 2], 8);
      record.writeFloatLE(mesh.positions[ib], 12); record.writeFloatLE(mesh.positions[ib + 1], 16); record.writeFloatLE(mesh.positions[ib + 2], 20);
      record.writeFloatLE(mesh.positions[ic], 24); record.writeFloatLE(mesh.positions[ic + 1], 28); record.writeFloatLE(mesh.positions[ic + 2], 32);
      record.writeUInt32LE(surface.id, 36); collisionRecords.push(record); collisionTriangleCount++;
    }
  }

  if (collisionTriangleCount === 0) throw new Error('No driveable collision triangles were detected. Check surfaces.ini or mesh naming before importing this track.');
  await writeFile(join(options.outputDir, 'collision.bin'), Buffer.concat(collisionRecords));

  const fallbackBounds = [0, 0, 0];
  const manifest = {
    format: 'physics-drive-track', version: 1, id: options.id, name: options.name,
    source: {
      type: 'assetto-corsa-kn5',
      model: options.modelFilename,
      models: options.sourceModels ?? [options.modelFilename],
      collisionModel: options.collisionModelFilename ?? options.modelFilename,
      kn5Version: visualModel.version,
    },
    visualBinary: 'visual.bin', collisionBinary: 'collision.bin', collisionTriangleCount,
    rejectedSteepTriangleCount,
    meshes: meshManifest, materials: visualModel.materials.map(packageMaterial), surfaces: packageSurfaces,
    spawnPoints: [{ name: 'default', x: options.spawn?.x ?? 0, z: options.spawn?.z ?? 0, yaw: options.spawn?.yaw ?? 0 }],
    bounds: { min: boundsMin.every(Number.isFinite) ? boundsMin : fallbackBounds, max: boundsMax.every(Number.isFinite) ? boundsMax : fallbackBounds },
  };
  await writeFile(join(options.outputDir, 'track.json'), `${JSON.stringify(manifest, null, 2)}\n`, 'utf8');
  return manifest;
}

export function defaultTrackId(sourcePath: string): string {
  return basename(sourcePath, extname(sourcePath)).replace(/^models_/i, '').toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || 'imported-track';
}

export function defaultOutputDirectory(sourcePath: string, id: string): string {
  return join(dirname(sourcePath), 'physics-drive-export', id);
}
