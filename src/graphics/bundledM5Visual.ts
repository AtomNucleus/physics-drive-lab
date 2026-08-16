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

  // Assetto Corsa uses +X left while Physics Drive Lab uses +X right.
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

export async function loadBundledM5Visual(config: VehicleConfig): Promise<Kn5VisualResult> {
  const data = await loadBundledKn5Bytes();
  const visual = await loadKn5Visual({ name: 'bmw_m5_2024_default_lod.kn5', data });
  alignToCurrentPhysics(visual, config);
  visual.warnings.push(
    'The default startup BMW uses the supplied M5 G90 mod lightweight LOD geometry. Importing the original KN5 still replaces it with the full-detail model.'
  );
  return visual;
}
