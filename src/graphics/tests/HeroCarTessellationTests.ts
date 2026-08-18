import assert from 'node:assert/strict';
import * as THREE from 'three';
import { enhanceHeroCarGeometry } from '../heroCarTessellation';

function assertClose(actual: number, expected: number, tolerance = 1e-6) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `Expected ${actual} to be within ${tolerance} of ${expected}`);
}

function boundsOf(geometry: THREE.BufferGeometry) {
  geometry.computeBoundingBox();
  assert.ok(geometry.boundingBox);
  return geometry.boundingBox!.clone();
}

{
  const root = new THREE.Group();
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(2, 1, 4), new THREE.MeshStandardMaterial());
  root.add(mesh);
  const before = boundsOf(mesh.geometry);

  const report = enhanceHeroCarGeometry(root, {
    targetTriangles: 48,
    triangleBudget: 48,
    maxPassesPerMesh: 1,
    minTrianglesPerMesh: 1,
  });

  assert.equal(report.sourceTriangles, 12, 'BoxGeometry should begin with 12 triangles');
  assert.equal(report.outputTriangles, 48, 'One midpoint subdivision pass must create 4x triangles');
  assert.equal(report.tessellatedMeshes, 1);
  assert.equal(report.tessellationPasses, 1);
  assert.equal(mesh.geometry.index?.count, 48 * 3);

  const after = boundsOf(mesh.geometry);
  assertClose(after.min.x, before.min.x);
  assertClose(after.min.y, before.min.y);
  assertClose(after.min.z, before.min.z);
  assertClose(after.max.x, before.max.x);
  assertClose(after.max.y, before.max.y);
  assertClose(after.max.z, before.max.z);
}

{
  const root = new THREE.Group();
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(2, 1, 4), new THREE.MeshStandardMaterial());
  root.add(mesh);

  const report = enhanceHeroCarGeometry(root, {
    targetTriangles: 192,
    triangleBudget: 48,
    maxPassesPerMesh: 2,
    minTrianglesPerMesh: 1,
  });

  assert.equal(report.triangleBudget, 48, 'The requested hard budget must remain authoritative');
  assert.equal(report.targetTriangles, 48, 'Target must clamp to the hard budget');
  assert.equal(report.outputTriangles, 48, 'The second pass must not exceed the hard budget');
}

console.log('Hero car tessellation tests passed.');
