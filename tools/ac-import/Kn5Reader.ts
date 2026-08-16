import { readFile } from 'node:fs/promises';

export interface Kn5Texture { type: number; name: string; data: Buffer; }
export interface Kn5Material { id: number; name: string; shader: string; properties: Record<string, number>; textures: Record<string, string>; }
export interface Kn5Mesh { name: string; materialId: number; positions: Float32Array; normals: Float32Array; uvs: Float32Array; indices: Uint32Array; }
export interface Kn5Model { version: number; textures: Kn5Texture[]; materials: Kn5Material[]; meshes: Kn5Mesh[]; }

type Mat4 = [number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number];
const IDENTITY: Mat4 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

class BinaryCursor {
  public offset = 0;
  constructor(public readonly buffer: Buffer) {}
  private ensure(bytes: number) {
    if (this.offset + bytes > this.buffer.length) throw new Error(`Unexpected end of KN5 at byte ${this.offset}; need ${bytes} more bytes.`);
  }
  readInt32() { this.ensure(4); const value = this.buffer.readInt32LE(this.offset); this.offset += 4; return value; }
  readUInt16() { this.ensure(2); const value = this.buffer.readUInt16LE(this.offset); this.offset += 2; return value; }
  readInt16() { this.ensure(2); const value = this.buffer.readInt16LE(this.offset); this.offset += 2; return value; }
  readUInt8() { this.ensure(1); return this.buffer[this.offset++]; }
  readFloat32() { this.ensure(4); const value = this.buffer.readFloatLE(this.offset); this.offset += 4; return value; }
  readBytes(length: number) {
    if (length < 0) throw new Error(`Invalid negative KN5 byte length ${length} at ${this.offset}.`);
    this.ensure(length); const value = this.buffer.subarray(this.offset, this.offset + length); this.offset += length; return value;
  }
  skip(length: number) { this.readBytes(length); }
  readFixedString(length: number) { return this.readBytes(length).toString('utf8'); }
  readString32() {
    const length = this.readInt32();
    if (length < 0 || length > 16 * 1024 * 1024) throw new Error(`Invalid KN5 string length ${length} at byte ${this.offset - 4}.`);
    return this.readFixedString(length);
  }
}

function multiplyRowMajor(a: Mat4, b: Mat4): Mat4 {
  const out = new Array<number>(16).fill(0) as unknown as Mat4;
  for (let row = 0; row < 4; row++) for (let col = 0; col < 4; col++) {
    out[row * 4 + col] = a[row * 4] * b[col] + a[row * 4 + 1] * b[4 + col] + a[row * 4 + 2] * b[8 + col] + a[row * 4 + 3] * b[12 + col];
  }
  return out;
}

function transformPoint(matrix: Mat4, x: number, y: number, z: number): [number, number, number] {
  return [
    x * matrix[0] + y * matrix[4] + z * matrix[8] + matrix[12],
    x * matrix[1] + y * matrix[5] + z * matrix[9] + matrix[13],
    x * matrix[2] + y * matrix[6] + z * matrix[10] + matrix[14],
  ];
}

function transformNormal(matrix: Mat4, x: number, y: number, z: number): [number, number, number] {
  const nx = x * matrix[0] + y * matrix[4] + z * matrix[8];
  const ny = x * matrix[1] + y * matrix[5] + z * matrix[9];
  const nz = x * matrix[2] + y * matrix[6] + z * matrix[10];
  const length = Math.hypot(nx, ny, nz) || 1;
  return [nx / length, ny / length, nz / length];
}

function readMatrix(cursor: BinaryCursor): Mat4 {
  const matrix = new Array<number>(16) as unknown as Mat4;
  for (let i = 0; i < 16; i++) matrix[i] = cursor.readFloat32();
  return matrix;
}

function readMeshPayload(cursor: BinaryCursor, name: string, worldMatrix: Mat4, animated: boolean): Kn5Mesh {
  cursor.skip(3);
  if (animated) {
    const boneCount = cursor.readInt32();
    if (boneCount < 0 || boneCount > 100000) throw new Error(`Invalid bone count ${boneCount} in ${name}.`);
    for (let bone = 0; bone < boneCount; bone++) { cursor.readString32(); cursor.skip(64); }
  }

  const vertexCount = cursor.readInt32();
  if (vertexCount < 0 || vertexCount > 20_000_000) throw new Error(`Invalid vertex count ${vertexCount} in ${name}.`);
  const positions = new Float32Array(vertexCount * 3);
  const normals = new Float32Array(vertexCount * 3);
  const uvs = new Float32Array(vertexCount * 2);

  for (let vertex = 0; vertex < vertexCount; vertex++) {
    const x = cursor.readFloat32(), y = cursor.readFloat32(), z = cursor.readFloat32();
    const nx = cursor.readFloat32(), ny = cursor.readFloat32(), nz = cursor.readFloat32();
    const u = cursor.readFloat32(), v = cursor.readFloat32();
    const p = transformPoint(worldMatrix, x, y, z);
    const n = transformNormal(worldMatrix, nx, ny, nz);
    positions.set(p, vertex * 3); normals.set(n, vertex * 3);
    uvs[vertex * 2] = u; uvs[vertex * 2 + 1] = 1 - v;
    cursor.skip(animated ? 44 : 12);
  }

  const indexCount = cursor.readInt32();
  if (indexCount < 0 || indexCount > 60_000_000) throw new Error(`Invalid index count ${indexCount} in ${name}.`);
  const indices = new Uint32Array(indexCount);
  for (let i = 0; i < indexCount; i++) indices[i] = cursor.readUInt16();
  const materialId = cursor.readInt32();
  cursor.skip(animated ? 12 : 29);
  return { name, materialId, positions, normals, uvs, indices };
}

function readNode(cursor: BinaryCursor, parentWorld: Mat4, meshes: Kn5Mesh[]) {
  const type = cursor.readInt32();
  const name = cursor.readString32();
  const childCount = cursor.readInt32();
  cursor.readUInt8();
  let worldMatrix = parentWorld;

  if (type === 1) worldMatrix = multiplyRowMajor(readMatrix(cursor), parentWorld);
  else if (type === 2) meshes.push(readMeshPayload(cursor, name, worldMatrix, false));
  else if (type === 3) meshes.push(readMeshPayload(cursor, name, worldMatrix, true));
  else throw new Error(`Unsupported KN5 node type ${type} for node "${name}" at byte ${cursor.offset}.`);

  if (childCount < 0 || childCount > 1_000_000) throw new Error(`Invalid child count ${childCount} for ${name}.`);
  for (let child = 0; child < childCount; child++) readNode(cursor, worldMatrix, meshes);
}

export function parseKn5(buffer: Buffer): Kn5Model {
  const cursor = new BinaryCursor(buffer);
  const magic = cursor.readFixedString(6);
  if (magic !== 'sc6969') throw new Error(`Not a KN5 file: expected sc6969, got ${JSON.stringify(magic)}.`);
  const version = cursor.readInt32();
  if (version < 1 || version > 50) throw new Error(`Unsupported or corrupt KN5 version ${version}.`);
  if (version > 5) cursor.readInt32();

  const textureCount = cursor.readInt32();
  if (textureCount < 0 || textureCount > 100000) throw new Error(`Invalid texture count ${textureCount}.`);
  const textures: Kn5Texture[] = [];
  for (let i = 0; i < textureCount; i++) {
    const type = cursor.readInt32(); const name = cursor.readString32(); const size = cursor.readInt32();
    textures.push({ type, name, data: Buffer.from(cursor.readBytes(size)) });
  }

  const materialCount = cursor.readInt32();
  if (materialCount < 0 || materialCount > 100000) throw new Error(`Invalid material count ${materialCount}.`);
  const materials: Kn5Material[] = [];
  for (let id = 0; id < materialCount; id++) {
    const name = cursor.readString32(); const shader = cursor.readString32();
    cursor.readInt16(); if (version > 4) cursor.readInt32();
    const propertyCount = cursor.readInt32();
    if (propertyCount < 0 || propertyCount > 100000) throw new Error(`Invalid property count ${propertyCount} in material ${name}.`);
    const properties: Record<string, number> = {};
    for (let p = 0; p < propertyCount; p++) { const propertyName = cursor.readString32(); properties[propertyName] = cursor.readFloat32(); cursor.skip(36); }
    const textureSlotCount = cursor.readInt32();
    if (textureSlotCount < 0 || textureSlotCount > 100000) throw new Error(`Invalid texture slot count ${textureSlotCount} in material ${name}.`);
    const materialTextures: Record<string, string> = {};
    for (let t = 0; t < textureSlotCount; t++) { const samplerName = cursor.readString32(); cursor.readInt32(); materialTextures[samplerName] = cursor.readString32(); }
    materials.push({ id, name, shader, properties, textures: materialTextures });
  }

  const meshes: Kn5Mesh[] = [];
  if (cursor.offset < buffer.length) readNode(cursor, IDENTITY, meshes);
  return { version, textures, materials, meshes };
}

export async function readKn5File(path: string): Promise<Kn5Model> { return parseKn5(await readFile(path)); }
