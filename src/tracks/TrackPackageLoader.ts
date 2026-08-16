import * as THREE from 'three';
import { DDSLoader } from 'three/addons/loaders/DDSLoader.js';
import { TrackPackageManifest } from './TrackTypes';
import { decodeCollisionTriangles, TriangleSurfaceProvider } from './TriangleSurfaceProvider';

export interface LoadedTrackPackage {
  manifest: TrackPackageManifest;
  surfaceProvider: TriangleSurfaceProvider;
  visualRoot: THREE.Group;
}

function resolveAsset(baseUrl: string, asset: string): string {
  return new URL(asset, baseUrl).toString();
}

async function fetchChecked(url: string): Promise<Response> {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`Failed to load ${url}: ${response.status} ${response.statusText}`);
  return response;
}

export async function loadTrackPackage(manifestUrl: string): Promise<LoadedTrackPackage> {
  const manifestResponse = await fetchChecked(manifestUrl);
  const manifest = await manifestResponse.json() as TrackPackageManifest;

  if (manifest.format !== 'physics-drive-track' || manifest.version !== 1) {
    throw new Error(`Unsupported track package format/version: ${manifest.format} v${manifest.version}`);
  }

  const [collisionBuffer, visualBuffer] = await Promise.all([
    fetchChecked(resolveAsset(manifestUrl, manifest.collisionBinary)).then((response) => response.arrayBuffer()),
    fetchChecked(resolveAsset(manifestUrl, manifest.visualBinary)).then((response) => response.arrayBuffer()),
  ]);

  const collisionTriangles = decodeCollisionTriangles(collisionBuffer, manifest.collisionTriangleCount);
  const surfaceProvider = new TriangleSurfaceProvider(collisionTriangles, manifest.surfaces);
  const visualRoot = await buildVisualRoot(manifestUrl, manifest, visualBuffer);

  return { manifest, surfaceProvider, visualRoot };
}

async function buildVisualRoot(manifestUrl: string, manifest: TrackPackageManifest, visualBuffer: ArrayBuffer): Promise<THREE.Group> {
  const root = new THREE.Group();
  root.name = `track:${manifest.id}`;
  const textureLoader = new THREE.TextureLoader();
  const ddsLoader = new DDSLoader();
  const materialCache = new Map<number, THREE.MeshStandardMaterial>();

  const loadTexture = (asset: string, colorTexture: boolean): THREE.Texture => {
    const url = resolveAsset(manifestUrl, asset);
    const texture = /\.dds(?:$|[?#])/i.test(url) ? ddsLoader.load(url) : textureLoader.load(url);
    if (colorTexture) texture.colorSpace = THREE.SRGBColorSpace;
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    return texture;
  };

  const getMaterial = (materialId: number): THREE.MeshStandardMaterial => {
    const cached = materialCache.get(materialId);
    if (cached) return cached;

    const source = manifest.materials.find((material) => material.id === materialId);
    const material = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.82, metalness: 0.03 });

    if (source?.diffuseMultiplier !== undefined) {
      const value = Math.max(0, Math.min(2, source.diffuseMultiplier));
      material.color.setRGB(Math.min(1, value), Math.min(1, value), Math.min(1, value));
    }

    if (source?.diffuseTexture) material.map = loadTexture(source.diffuseTexture, true);
    if (source?.normalTexture) material.normalMap = loadTexture(source.normalTexture, false);

    materialCache.set(materialId, material);
    return material;
  };

  for (const meshInfo of manifest.meshes) {
    const geometry = new THREE.BufferGeometry();
    const positions = new Float32Array(visualBuffer, meshInfo.positions.byteOffset, meshInfo.positions.count * 3);
    const normals = new Float32Array(visualBuffer, meshInfo.normals.byteOffset, meshInfo.normals.count * 3);
    const uvs = new Float32Array(visualBuffer, meshInfo.uvs.byteOffset, meshInfo.uvs.count * 2);
    const indices = new Uint32Array(visualBuffer, meshInfo.indices.byteOffset, meshInfo.indices.count);

    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
    geometry.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    geometry.setIndex(new THREE.BufferAttribute(indices, 1));
    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();

    const mesh = new THREE.Mesh(geometry, getMaterial(meshInfo.materialId));
    mesh.name = meshInfo.name;
    mesh.receiveShadow = true;
    mesh.castShadow = false;
    root.add(mesh);
  }

  return root;
}
