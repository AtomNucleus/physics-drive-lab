import * as THREE from 'three';
import type { VehicleConfig } from '../types';
import { alignKn5ToCurrentPhysics } from './kn5VisualAlignment';
import { loadKn5Visual, type Kn5VisualResult } from './kn5Loader';
import { fitM5VisualToRealScale } from './m5VisualScale';

const FULL_M5_ASSET_DIR = `${import.meta.env.BASE_URL}assets/bmw-m5-g90-full`;
const GLASS_SEAM_MAX_MARGIN_M = 0.018;
const GLASS_SEAM_EXPANSION_RATIO = 0.04;
const GLASS_SEAM_MIN_COMPONENT_SIZE_M = 0.08;

interface FullM5Manifest {
  format: 'kn5-gzip-binary-v1';
  quality: 'full';
  modelFile: string;
  parts: number;
  partBytes: number;
  kn5Bytes: number;
  gzipBytes: number;
  sha256: string;
}

async function loadFullM5Bytes(): Promise<{ manifest: FullM5Manifest; data: Uint8Array }> {
  const manifestResponse = await fetch(`${FULL_M5_ASSET_DIR}/manifest.json`);
  if (!manifestResponse.ok) {
    throw new Error(`Full-quality BMW manifest failed to load (${manifestResponse.status}).`);
  }

  const manifest = (await manifestResponse.json()) as FullM5Manifest;
  if (
    manifest.format !== 'kn5-gzip-binary-v1' ||
    manifest.quality !== 'full' ||
    !Number.isInteger(manifest.parts) ||
    manifest.parts <= 0 ||
    !Number.isFinite(manifest.partBytes) ||
    manifest.partBytes <= 0 ||
    !manifest.modelFile.toLowerCase().endsWith('.kn5')
  ) {
    throw new Error('Bundled BMW full-quality manifest is invalid.');
  }

  const binaryParts = await Promise.all(
    Array.from({ length: manifest.parts }, async (_, index) => {
      const part = String(index).padStart(2, '0');
      const response = await fetch(`${FULL_M5_ASSET_DIR}/part-${part}.bin`);
      if (!response.ok) {
        throw new Error(`Full-quality BMW asset part ${part} failed to load (${response.status}).`);
      }
      return new Uint8Array(await response.arrayBuffer());
    })
  );

  const compressedLength = binaryParts.reduce((sum, part) => sum + part.byteLength, 0);
  if (compressedLength !== manifest.gzipBytes) {
    throw new Error(
      `Full-quality BMW asset size mismatch (${compressedLength} compressed bytes; expected ${manifest.gzipBytes}).`
    );
  }

  for (let index = 0; index < binaryParts.length - 1; index += 1) {
    if (binaryParts[index].byteLength !== manifest.partBytes) {
      throw new Error(
        `Full-quality BMW asset part ${String(index).padStart(2, '0')} has ${binaryParts[index].byteLength} bytes; expected ${manifest.partBytes}.`
      );
    }
  }

  const compressed = new Uint8Array(compressedLength);
  let offset = 0;
  for (const part of binaryParts) {
    compressed.set(part, offset);
    offset += part.byteLength;
  }

  const Decompression = (globalThis as any).DecompressionStream;
  if (!Decompression) {
    throw new Error('This browser does not support gzip decompression for the full-quality BMW visual.');
  }

  const stream = new Blob([compressed.slice().buffer]).stream().pipeThrough(new Decompression('gzip'));
  const data = new Uint8Array(await new Response(stream).arrayBuffer());
  if (data.byteLength !== manifest.kn5Bytes) {
    throw new Error(
      `Full-quality BMW KN5 size mismatch (${data.byteLength} bytes; expected ${manifest.kn5Bytes}).`
    );
  }

  return { manifest, data };
}

function materialName(material: THREE.Material): string {
  return material.name.toLowerCase().replace(/[^a-z0-9]+/g, '_');
}

function meshIdentity(mesh: THREE.Mesh, material: THREE.Material): string {
  return `${mesh.name} ${mesh.parent?.name ?? ''} ${material.name}`.toLowerCase().replace(/[^a-z0-9]+/g, '_');
}

function isCabinGlass(mesh: THREE.Mesh, material: THREE.Material): boolean {
  const identity = meshIdentity(mesh, material);
  const name = materialName(material);

  // Strong pane names win even when a rear/front qualifier is present.
  const strongPaneTokens = [
    'window',
    'windscreen',
    'windshield',
    'sideglass',
    'side_glass',
    'front_glass',
    'rear_glass',
    'glass_front',
    'glass_rear',
    'glass_int',
    'glass_ext',
    'cockpit_glass',
  ];
  if (strongPaneTokens.some((token) => identity.includes(token))) return true;

  if (!identity.includes('glass') && !name.includes('glass')) return false;

  // Do not turn lamp lenses, reflectors, or colored light glass into cabin glass.
  const lampTokens = [
    'headlight',
    'taillight',
    'tail_light',
    'brake_light',
    'indicator',
    'blinker',
    'turn_light',
    'reverse_light',
    'lamp',
    'reflector',
    'glass_red',
    'glass_orange',
  ];
  return !lampTokens.some((token) => identity.includes(token));
}

function buildGlassMaterial(source: THREE.Material): THREE.MeshPhysicalMaterial {
  const standard = source as THREE.MeshStandardMaterial;
  const sourceColor = standard.color instanceof THREE.Color ? standard.color.clone() : new THREE.Color(0x71889b);
  // Keep authored tint/texture while preventing near-black AC shader fallback from
  // making the cabin opaque in the browser renderer.
  sourceColor.lerp(new THREE.Color(0x8095a8), 0.28);

  return new THREE.MeshPhysicalMaterial({
    name: source.name || 'cabin_glass',
    color: sourceColor,
    map: standard.map ?? null,
    normalMap: standard.normalMap ?? null,
    roughness: 0.07,
    metalness: 0,
    transparent: true,
    opacity: 0.42,
    depthWrite: false,
    side: THREE.DoubleSide,
    transmission: 0.08,
    ior: 1.5,
    thickness: 0.006,
    clearcoat: 1,
    clearcoatRoughness: 0.025,
    envMapIntensity: 1.3,
  });
}

/**
 * The real KN5 has separate glass surfaces. Expand each connected pane only in
 * its own tangent plane so its perimeter sits just underneath the surrounding
 * frame/body. No triangles are added and the glass is never pushed outward.
 */
function sealGlassPerimeter(geometry: THREE.BufferGeometry): void {
  const position = geometry.getAttribute('position') as THREE.BufferAttribute | undefined;
  const index = geometry.getIndex();
  if (!position || position.count < 3 || !index || index.count < 3) return;

  if (!geometry.getAttribute('normal')) geometry.computeVertexNormals();
  const normal = geometry.getAttribute('normal') as THREE.BufferAttribute | undefined;
  if (!normal) return;

  const adjacency = Array.from({ length: position.count }, () => new Set<number>());
  const used = new Uint8Array(position.count);
  for (let i = 0; i + 2 < index.count; i += 3) {
    const a = index.getX(i);
    const b = index.getX(i + 1);
    const c = index.getX(i + 2);
    used[a] = 1;
    used[b] = 1;
    used[c] = 1;
    adjacency[a].add(b); adjacency[a].add(c);
    adjacency[b].add(a); adjacency[b].add(c);
    adjacency[c].add(a); adjacency[c].add(b);
  }

  const visited = new Uint8Array(position.count);
  for (let seed = 0; seed < position.count; seed += 1) {
    if (!used[seed] || visited[seed]) continue;

    const component: number[] = [];
    const stack = [seed];
    visited[seed] = 1;
    while (stack.length > 0) {
      const vertex = stack.pop()!;
      component.push(vertex);
      for (const neighbor of adjacency[vertex]) {
        if (visited[neighbor]) continue;
        visited[neighbor] = 1;
        stack.push(neighbor);
      }
    }

    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    for (const vertex of component) {
      const x = position.getX(vertex);
      const y = position.getY(vertex);
      const z = position.getZ(vertex);
      minX = Math.min(minX, x); minY = Math.min(minY, y); minZ = Math.min(minZ, z);
      maxX = Math.max(maxX, x); maxY = Math.max(maxY, y); maxZ = Math.max(maxZ, z);
    }

    const componentSize = Math.max(maxX - minX, maxY - minY, maxZ - minZ);
    if (!Number.isFinite(componentSize) || componentSize < GLASS_SEAM_MIN_COMPONENT_SIZE_M) continue;

    const centerX = (minX + maxX) * 0.5;
    const centerY = (minY + maxY) * 0.5;
    const centerZ = (minZ + maxZ) * 0.5;

    for (const vertex of component) {
      const x = position.getX(vertex);
      const y = position.getY(vertex);
      const z = position.getZ(vertex);
      const nx = normal.getX(vertex);
      const ny = normal.getY(vertex);
      const nz = normal.getZ(vertex);

      const dx = x - centerX;
      const dy = y - centerY;
      const dz = z - centerZ;
      const normalProjection = dx * nx + dy * ny + dz * nz;
      const tx = dx - nx * normalProjection;
      const ty = dy - ny * normalProjection;
      const tz = dz - nz * normalProjection;
      const tangentLength = Math.hypot(tx, ty, tz);
      if (tangentLength < 1e-6) continue;

      const margin = Math.min(GLASS_SEAM_MAX_MARGIN_M, tangentLength * GLASS_SEAM_EXPANSION_RATIO);
      const scale = margin / tangentLength;
      position.setXYZ(vertex, x + tx * scale, y + ty * scale, z + tz * scale);
    }
  }

  position.needsUpdate = true;
  geometry.computeVertexNormals();
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
}

function upgradeRealGlassGeometry(root: THREE.Group): number {
  let glassMeshCount = 0;
  root.traverse((object) => {
    if (!(object instanceof THREE.Mesh)) return;

    const materials = Array.isArray(object.material) ? object.material : [object.material];
    let foundGlass = false;
    const upgraded = materials.map((material) => {
      if (!isCabinGlass(object, material)) return material;
      foundGlass = true;
      return buildGlassMaterial(material);
    });

    if (!foundGlass) return;
    sealGlassPerimeter(object.geometry);
    object.material = Array.isArray(object.material) ? upgraded : upgraded[0];
    object.renderOrder = Math.max(object.renderOrder, 3);
    object.castShadow = false;
    glassMeshCount += 1;
  });
  return glassMeshCount;
}

export async function loadBundledM5Visual(config: VehicleConfig): Promise<Kn5VisualResult> {
  const { manifest, data } = await loadFullM5Bytes();
  const visual = await loadKn5Visual({ name: manifest.modelFile, data });

  const scaleReport = fitM5VisualToRealScale(visual.group);
  alignKn5ToCurrentPhysics(visual, config as unknown as Record<string, any>);
  const glassMeshCount = upgradeRealGlassGeometry(visual.group);

  const scaleMessage = Math.abs(scaleReport.appliedScale - 1) > 0.0025
    ? `Full-quality BMW visual normalized from ${scaleReport.sourceLengthM.toFixed(3)} m to ${scaleReport.finalLengthM.toFixed(3)} m (x${scaleReport.appliedScale.toFixed(4)}).`
    : `Full-quality BMW visual metre scale verified at ${scaleReport.finalLengthM.toFixed(3)} m long.`;

  visual.warnings.push(
    `FULL QUALITY: ${visual.meshCount} imported KN5 meshes, ${visual.textureCount} decoded embedded textures, ${visual.materialCount} materials.`,
    `${glassMeshCount} real cabin-glass mesh${glassMeshCount === 1 ? '' : 'es'} use transparent physical glass with sealed frame overlap.`,
    'Visual mesh remains render-only: the current simple collider, calibrated chassis physics, suspension, tires, and physics-driven wheels remain authoritative.',
    scaleMessage,
  );

  if (glassMeshCount === 0) {
    visual.warnings.push('No cabin-glass mesh names matched the full-quality glass classifier; inspect the KN5 material naming.');
  }

  return visual;
}
