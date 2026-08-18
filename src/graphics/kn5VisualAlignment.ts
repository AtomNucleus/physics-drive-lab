import * as THREE from 'three';
import type { Kn5VisualResult } from './kn5Loader';

function findNodeByNames(root: THREE.Object3D, names: string[]): THREE.Object3D | null {
  const wanted = new Set(names.map((name) => name.toUpperCase()));
  let found: THREE.Object3D | null = null;
  root.traverse((object) => {
    if (!found && wanted.has(object.name.toUpperCase())) found = object;
  });
  return found;
}

/**
 * Align an imported Assetto Corsa KN5 body to the simulator's current axle/CG
 * reference frame. This is a visual transform only: the vehicle collider,
 * suspension, tires, mass properties, and physics-driven wheel assemblies remain
 * owned by Physics Drive Lab.
 */
export function alignKn5ToCurrentPhysics(visual: Kn5VisualResult, config: Record<string, any>): void {
  const root = visual.group;

  // The supplied G90 KN5 uses the opposite lateral convention from the runtime
  // render frame. Preserve any uniform scale already applied to the model while
  // mirroring only the lateral axis.
  root.scale.x = -Math.abs(root.scale.x || 1);
  root.updateMatrixWorld(true);

  const lf = findNodeByNames(root, ['WHEEL_LF', 'WHEEL_FL']);
  const rf = findNodeByNames(root, ['WHEEL_RF', 'WHEEL_FR']);
  const lr = findNodeByNames(root, ['WHEEL_LR', 'WHEEL_RL']);
  const rr = findNodeByNames(root, ['WHEEL_RR']);

  if (!lf || !rf || !lr || !rr) {
    visual.warnings.push('KN5 wheel anchors could not be resolved, so the body kept its authored origin.');
    return;
  }

  const pLF = lf.getWorldPosition(new THREE.Vector3());
  const pRF = rf.getWorldPosition(new THREE.Vector3());
  const pLR = lr.getWorldPosition(new THREE.Vector3());
  const pRR = rr.getWorldPosition(new THREE.Vector3());

  const acFrontZ = (pLF.z + pRF.z) * 0.5;
  const acRearZ = (pLR.z + pRR.z) * 0.5;
  const acWheelY = (pLF.y + pRF.y + pLR.y + pRR.y) * 0.25;
  const acWheelbase = Math.abs(acFrontZ - acRearZ);
  const acFrontTrack = Math.abs(pLF.x - pRF.x);
  const acRearTrack = Math.abs(pLR.x - pRR.x);

  const wheelbase = Number(config.wheelbase);
  const frontWeight = Number(config.weightDistributionFront);
  const wheelRadius = Number(config.wheelRadius);

  if (Number.isFinite(wheelbase) && wheelbase > 0 && Number.isFinite(frontWeight)) {
    // Simulator hardpoints are referenced to the vehicle CG, whereas KN5 origins
    // are arbitrary model-space origins. Align both axles, not the raw model origin.
    const simFrontZ = wheelbase * (1 - frontWeight);
    const simRearZ = -wheelbase * frontWeight;
    const zOffset = ((simFrontZ - acFrontZ) + (simRearZ - acRearZ)) * 0.5;
    root.position.z += zOffset;

    visual.warnings.push(
      `KN5 axle anchors aligned to the current physics origin (model wheelbase ${acWheelbase.toFixed(3)} m; longitudinal offset ${zOffset >= 0 ? '+' : ''}${zOffset.toFixed(3)} m).`
    );
  }

  if (Number.isFinite(wheelRadius) && wheelRadius > 0 && Number.isFinite(acWheelY)) {
    // Normally millimetre-scale for a matching car; this prevents a correct full
    // body from floating when the authored model origin differs from the physics CG.
    root.position.y += wheelRadius - acWheelY;
  }

  root.updateMatrixWorld(true);
  visual.warnings.push(
    `KN5 wheel anchors: front track ${acFrontTrack.toFixed(3)} m, rear track ${acRearTrack.toFixed(3)} m.`
  );
}
