export type TrackSurfaceKind =
  | 'asphalt'
  | 'racing_line'
  | 'kerb'
  | 'wet'
  | 'marbles'
  | 'gravel'
  | 'grass'
  | 'sand'
  | 'dirt'
  | 'concrete'
  | 'unknown';

export interface TrackSurfaceMaterial {
  id: number;
  key: string;
  name: string;
  type: TrackSurfaceKind;
  friction: number;
  rollingResistance: number;
  wetness: number;
  isKerbRumble: boolean;
  isValidTrack?: boolean;
}

export interface TrackSpawnPoint {
  name: string;
  x: number;
  z: number;
  yaw: number;
  y?: number;
}

export interface PackedTrackMesh {
  name: string;
  materialId: number;
  positions: { byteOffset: number; count: number };
  normals: { byteOffset: number; count: number };
  uvs: { byteOffset: number; count: number };
  indices: { byteOffset: number; count: number };
}

export interface PackedTrackMaterial {
  id: number;
  name: string;
  shader: string;
  diffuseTexture?: string;
  normalTexture?: string;
  diffuseMultiplier?: number;
  specular?: number;
  specularExponent?: number;
}

export interface TrackPackageManifest {
  format: 'physics-drive-track';
  version: 1;
  id: string;
  name: string;
  source: {
    type: 'assetto-corsa-kn5';
    model: string;
    models?: string[];
    collisionModel?: string;
    kn5Version: number;
  };
  visualBinary: string;
  collisionBinary: string;
  collisionTriangleCount: number;
  rejectedSteepTriangleCount?: number;
  rejectedObstacleTriangleCount?: number;
  meshes: PackedTrackMesh[];
  materials: PackedTrackMaterial[];
  surfaces: TrackSurfaceMaterial[];
  spawnPoints: TrackSpawnPoint[];
  bounds: {
    min: [number, number, number];
    max: [number, number, number];
  };
}
