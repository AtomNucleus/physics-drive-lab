import * as THREE from 'three';
import type { VehicleConfig } from '../types';
import { loadKn5Visual, type Kn5VisualResult } from './kn5Loader';

const DEFAULT_M5_ASSET_PARTS = 8;
const DEFAULT_M5_ASSET_DIR = `${import.meta.env.BASE_URL}assets/bmw-m5-g90-default`;

function findNodeByNames(root: THREE.Object3D, names: string[]): THREE.Object3D | null {
  const wanted = new Set(names.map((name) => name.toUpperCase()));
  let found: THREE.Object3D | null = null;
  root.traverse((object) => {
    if (!found && wanted.has(object.name.toUpperCase())) found = object;
  });
  return found;
}

async function loadBundledKn5Bytes(): Promise<Uint8Array> {
  const parts = await Promise.all(
    Array.from({ length: DEFAULT_M5_ASSET_PARTS }, async (_, index) => {
      const part = String(index).padStart(2, '0');
      const response = await fetch(`${DEFAULT_M5_ASSET_DIR}/part-${part}.b64`);
      if (!response.ok) {
        throw new Error(`Default BMW asset part ${part} failed to load (${response.status}).`);
      }
      return (await response.text()).trim();
    })
  );

  const binary = atob(parts.join('').replace(/\s+/g, ''));
  const compressed = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i += 1) compressed[i] = binary.charCodeAt(i);

  const Ctor = (globalThis as any).DecompressionStream;
  if (!Ctor) {
    throw new Error('This browser does not support gzip decompression for the bundled BMW visual.');
  }
  const stream = new Blob([compressed.slice().buffer]).stream().pipeThrough(new Ctor('gzip'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

function alignToCurrentPhysics(visual: Kn5VisualResult, config: VehicleConfig) {
  const root = visual.group;

  // The supplied AC G90 is authored with +Z toward the hood/front axle, which
  // matches Physics Drive Lab. Only the lateral axis needs mirroring because AC's
  // +X is the car's left while our +X is the car's right.
  root.scale.x = -1;
  root.updateMatrixWorld(true);

  const lf = findNodeByNames(root, ['WHEEL_LF', 'WHEEL_FL']);
  const rf = findNodeByNames(root, ['WHEEL_RF', 'WHEEL_FR']);
  const lr = findNodeByNames(root, ['WHEEL_LR', 'WHEEL_RL']);
  const rr = findNodeByNames(root, ['WHEEL_RR']);

  if (!lf || !rf || !lr || !rr) {
    visual.warnings.push('Bundled BMW wheel anchors could not be resolved; the authored body origin was retained.');
    return;
  }

  const pLF = lf.getWorldPosition(new THREE.Vector3());
  const pRF = rf.getWorldPosition(new THREE.Vector3());
  const pLR = lr.getWorldPosition(new THREE.Vector3());
  const pRR = rr.getWorldPosition(new THREE.Vector3());

  const acFrontZ = (pLF.z + pRF.z) * 0.5;
  const acRearZ = (pLR.z + pRR.z) * 0.5;
  const acWheelY = (pLF.y + pRF.y + pLR.y + pRR.y) * 0.25;

  const wheelbase = Number(config.wheelbase);
  const frontWeight = Number(config.weightDistributionFront);
  const wheelRadius = Number(config.wheelRadius);

  if (Number.isFinite(wheelbase) && wheelbase > 0 && Number.isFinite(frontWeight)) {
    const simFrontZ = wheelbase * (1 - frontWeight);
    const simRearZ = -wheelbase * frontWeight;
    root.position.z += ((simFrontZ - acFrontZ) + (simRearZ - acRearZ)) * 0.5;
  }

  if (Number.isFinite(wheelRadius) && wheelRadius > 0 && Number.isFinite(acWheelY)) {
    root.position.y += wheelRadius - acWheelY;
  }

  root.updateMatrixWorld(true);
}

function tuneBundledFallbackMaterials(visual: Kn5VisualResult) {
  // The tiny startup KN5 intentionally contains no embedded bitmap textures. Do
  // not let the generic KN5 neutral-gray fallback make glass opaque or turn every
  // authored material into the same surface. Preserve the mod's material names
  // and approximate their physical appearance until the full KN5 is imported.
  const seen = new Set<THREE.Material>();

  visual.group.traverse((object) => {
    if (!(object instanceof THREE.Mesh)) return;
    const materials = Array.isArray(object.material) ? object.material : [object.material];

    for (const material of materials) {
      if (!material || seen.has(material)) continue;
      seen.add(material);
      if (!(material instanceof THREE.MeshStandardMaterial)) continue;

      const name = material.name.toLowerCase();
      material.map = null;
      material.normalMap = null;
      material.alphaMap = null;

      if (name === 'carpaint') {
        material.color.set(0x18202b);
        material.metalness = 0.72;
        material.roughness = 0.2;
        material.envMapIntensity = 1.35;
      } else if (name === 'window' || name === 'glass_int') {
        material.color.set(0x101b2a);
        material.metalness = 0.04;
        material.roughness = 0.08;
        material.transparent = true;
        material.opacity = name === 'window' ? 0.28 : 0.16;
        material.depthWrite = false;
        material.side = THREE.DoubleSide;
      } else if (name === 'glass_red') {
        material.color.set(0x8b0b16);
        material.emissive.set(0x310208);
        material.emissiveIntensity = 0.55;
        material.metalness = 0.02;
        material.roughness = 0.18;
        material.transparent = true;
        material.opacity = 0.82;
        material.depthWrite = false;
      } else if (name.includes('light')) {
        material.color.set(0xdbeafe);
        material.emissive.set(0xb9d9ff);
        material.emissiveIntensity = 0.55;
        material.metalness = 0.2;
        material.roughness = 0.16;
      } else if (name.includes('chrome') || name.includes('badge')) {
        material.color.set(0xcbd5e1);
        material.metalness = 0.92;
        material.roughness = 0.12;
      } else if (name.includes('grille') || name.includes('black') || name.includes('carbon') || name === 'roof' || name === 'wiper') {
        material.color.set(name.includes('carbon') ? 0x111318 : 0x090b0f);
        material.metalness = name.includes('gloss') ? 0.42 : 0.24;
        material.roughness = name.includes('gloss') ? 0.2 : 0.42;
      } else if (name.includes('plate')) {
        material.color.set(0xe5e7eb);
        material.metalness = 0.05;
        material.roughness = 0.55;
      } else if (name.includes('leather') || name.includes('interior') || name.includes('carpet') || name.includes('int_')) {
        material.color.set(0x15181e);
        material.metalness = 0.03;
        material.roughness = 0.78;
      } else {
        material.color.set(0x20252c);
        material.metalness = 0.18;
        material.roughness = 0.48;
      }

      material.needsUpdate = true;
    }
  });
}

export async function loadBundledM5Visual(config: VehicleConfig): Promise<Kn5VisualResult> {
  const data = await loadBundledKn5Bytes();
  const visual = await loadKn5Visual({ name: 'bmw_m5_2024_default_lod.kn5', data });
  tuneBundledFallbackMaterials(visual);
  alignToCurrentPhysics(visual, config);
  visual.warnings.push(
    'The default startup BMW uses the supplied M5 G90 lightweight LOD geometry with material-name fallbacks. Importing the original KN5 replaces it with the full-detail textured model.'
  );
  return visual;
}
