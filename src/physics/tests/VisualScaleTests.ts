import * as THREE from 'three';
import { BMW_M5_2025_OVERRIDES } from '../m5G90';
import {
  horizontalFovForVertical,
  targetHorizontalFov,
  targetVerticalFov,
} from '../../graphics/cameraProjection';
import { fitM5VisualToRealScale } from '../../graphics/m5VisualScale';
import {
  BASE_VISUAL_WHEEL_RADIUS_M,
  BMW_M5_G90_LENGTH_M,
  MAIN_TEST_LANE_WIDTH_M,
} from '../../graphics/worldScale';

const assert = (condition: boolean, message: string) => {
  if (!condition) throw new Error(message);
};

const approx = (actual: number, expected: number, tolerance: number, message: string) => {
  assert(Math.abs(actual - expected) <= tolerance, `${message}: expected ${expected}, got ${actual}`);
};

const aspect16x9 = 16 / 9;
const aspect21x9 = 21 / 9;
const speedKmh = 50;

// Legacy chase projection at 50 km/h was 66 deg V-FOV. On a normal widescreen
// monitor that is already ~98 deg H-FOV, and on ultrawide it exceeds 113 deg.
const legacyHorizontal16x9 = horizontalFovForVertical(66, aspect16x9);
const legacyHorizontal21x9 = horizontalFovForVertical(66, aspect21x9);
assert(legacyHorizontal16x9 > 98, `legacy 16:9 H-FOV unexpectedly small: ${legacyHorizontal16x9}`);
assert(legacyHorizontal21x9 > 113, `legacy 21:9 H-FOV unexpectedly small: ${legacyHorizontal21x9}`);

// New chase projection is authored as H-FOV and therefore keeps the same visual
// distance scale on every aspect ratio.
const targetHorizontal = targetHorizontalFov('chase', speedKmh);
approx(targetHorizontal, 82, 1e-9, '50 km/h chase H-FOV');
const vertical16x9 = targetVerticalFov('chase', speedKmh, aspect16x9);
const vertical21x9 = targetVerticalFov('chase', speedKmh, aspect21x9);
approx(horizontalFovForVertical(vertical16x9, aspect16x9), targetHorizontal, 1e-9, '16:9 projection round-trip');
approx(horizontalFovForVertical(vertical21x9, aspect21x9), targetHorizontal, 1e-9, '21:9 projection round-trip');
assert(vertical16x9 < 53 && vertical16x9 > 51, `16:9 vertical FOV should be ~52 deg, got ${vertical16x9}`);

// The M5 physical wheel radius is 0.369 m; the procedural mesh was authored at
// 0.33 m. Runtime scaling must therefore enlarge it by ~11.8%, not leave it small.
const physicalWheelRadius = BMW_M5_2025_OVERRIDES.wheelRadius as number;
const wheelVisualScale = physicalWheelRadius / BASE_VISUAL_WHEEL_RADIUS_M;
approx(wheelVisualScale, 1.1181818181818182, 1e-9, 'M5 wheel visual scale');

// The visible central test lane must match the surface provider's |x| <= 6.5 m zone.
approx(MAIN_TEST_LANE_WIDTH_M, 13, 1e-9, 'main test lane width');

// A 20-23 m radius is not hundreds of metres: its diameter is ~7.85-9.03 G90
// body lengths. This gives a stable visual sanity check independent of camera.
const minDiameterCarLengths = (20 * 2) / BMW_M5_G90_LENGTH_M;
const maxDiameterCarLengths = (23 * 2) / BMW_M5_G90_LENGTH_M;
assert(minDiameterCarLengths > 7.8 && minDiameterCarLengths < 7.9, `20 m radius scale ratio invalid: ${minDiameterCarLengths}`);
assert(maxDiameterCarLengths > 9.0 && maxDiameterCarLengths < 9.1, `23 m radius scale ratio invalid: ${maxDiameterCarLengths}`);

// Validate the body-scale guard with a deliberately undersized synthetic asset.
const mockM5 = new THREE.Group();
mockM5.add(new THREE.Mesh(new THREE.BoxGeometry(1.8, 1.4, 4.6), new THREE.MeshBasicMaterial()));
const scaleReport = fitM5VisualToRealScale(mockM5);
approx(scaleReport.finalLengthM, BMW_M5_G90_LENGTH_M, 1e-6, 'normalized M5 body length');
assert(scaleReport.appliedScale > 1.10 && scaleReport.appliedScale < 1.12, `unexpected body scale factor: ${scaleReport.appliedScale}`);

console.log(JSON.stringify({
  speedKmh,
  camera: {
    legacyHorizontal16x9Deg: legacyHorizontal16x9,
    legacyHorizontal21x9Deg: legacyHorizontal21x9,
    correctedHorizontalDeg: targetHorizontal,
    correctedVertical16x9Deg: vertical16x9,
    correctedVertical21x9Deg: vertical21x9,
  },
  worldScale: {
    physicalWheelRadiusM: physicalWheelRadius,
    authoredWheelRadiusM: BASE_VISUAL_WHEEL_RADIUS_M,
    wheelVisualScale,
    mainTestLaneWidthM: MAIN_TEST_LANE_WIDTH_M,
    turnDiameterCarLengths: [minDiameterCarLengths, maxDiameterCarLengths],
    normalizedBodyLengthM: scaleReport.finalLengthM,
  },
  status: 'passed',
}, null, 2));
