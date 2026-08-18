import { createHash } from 'node:crypto';
import { gunzipSync } from 'node:zlib';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const root = resolve(process.cwd(), 'public/assets/bmw-m5-g90-full');
const manifest = JSON.parse(readFileSync(resolve(root, 'manifest.json'), 'utf8'));

if (manifest.format !== 'kn5-gzip-binary-v1' || manifest.quality !== 'full') {
  throw new Error('Full-quality M5 manifest format/quality is invalid.');
}
if (!Number.isInteger(manifest.parts) || manifest.parts <= 0) {
  throw new Error(`Invalid full-quality M5 part count: ${manifest.parts}`);
}
if (!Number.isInteger(manifest.partBytes) || manifest.partBytes <= 0) {
  throw new Error(`Invalid full-quality M5 part size: ${manifest.partBytes}`);
}

const chunks = [];
for (let index = 0; index < manifest.parts; index += 1) {
  const part = String(index).padStart(2, '0');
  const chunk = readFileSync(resolve(root, `part-${part}.bin`));
  if (index < manifest.parts - 1 && chunk.byteLength !== manifest.partBytes) {
    throw new Error(`Binary part ${part} size mismatch: ${chunk.byteLength} != ${manifest.partBytes}`);
  }
  chunks.push(chunk);
}

const compressed = Buffer.concat(chunks);
if (compressed.byteLength !== manifest.gzipBytes) {
  throw new Error(`Compressed full-quality M5 size mismatch: ${compressed.byteLength} != ${manifest.gzipBytes}`);
}

const kn5 = gunzipSync(compressed);
if (kn5.byteLength !== manifest.kn5Bytes) {
  throw new Error(`Full-quality M5 KN5 size mismatch: ${kn5.byteLength} != ${manifest.kn5Bytes}`);
}
if (kn5.subarray(0, 6).toString('ascii') !== 'sc6969') {
  throw new Error('Full-quality M5 asset is not a valid KN5 (bad magic).');
}

const version = kn5.readInt32LE(6);
if (version !== 6) {
  throw new Error(`Expected KN5 version 6 for the full G90 model; got ${version}.`);
}

const sha256 = createHash('sha256').update(kn5).digest('hex');
if (sha256 !== manifest.sha256) {
  throw new Error(`Full-quality M5 SHA-256 mismatch: ${sha256} != ${manifest.sha256}`);
}

class Reader {
  constructor(buffer, offset = 0) {
    this.buffer = buffer;
    this.offset = offset;
  }
  ensure(size) {
    if (this.offset + size > this.buffer.length) throw new Error(`Unexpected KN5 EOF at ${this.offset}`);
  }
  i16() { this.ensure(2); const value = this.buffer.readInt16LE(this.offset); this.offset += 2; return value; }
  i32() { this.ensure(4); const value = this.buffer.readInt32LE(this.offset); this.offset += 4; return value; }
  f32() { this.ensure(4); const value = this.buffer.readFloatLE(this.offset); this.offset += 4; return value; }
  skip(size) { this.ensure(size); this.offset += size; }
  string() {
    const length = this.i32();
    if (length < 0 || length > 16_000_000) throw new Error(`Invalid KN5 string length ${length}`);
    this.ensure(length);
    const value = this.buffer.subarray(this.offset, this.offset + length).toString('utf8');
    this.offset += length;
    return value;
  }
}

const reader = new Reader(kn5, 10);
if (version > 5) reader.skip(4);
const textureCount = reader.i32();
if (textureCount < 0 || textureCount > 10000) throw new Error(`Invalid texture count ${textureCount}`);
for (let index = 0; index < textureCount; index += 1) {
  const type = reader.i32();
  reader.string();
  if (type !== 0) {
    const size = reader.i32();
    if (size < 0) throw new Error(`Invalid texture size ${size}`);
    reader.skip(size);
  }
}

const materialCount = reader.i32();
if (materialCount < 0 || materialCount > 10000) throw new Error(`Invalid material count ${materialCount}`);
const materialNames = [];
for (let index = 0; index < materialCount; index += 1) {
  const name = reader.string();
  const shader = reader.string();
  reader.i16();
  if (version > 4) reader.skip(4);

  const propertyCount = reader.i32();
  if (propertyCount < 0 || propertyCount > 10000) throw new Error(`Invalid property count ${propertyCount}`);
  for (let prop = 0; prop < propertyCount; prop += 1) {
    reader.string();
    reader.f32();
    reader.skip(36);
  }

  const slotCount = reader.i32();
  if (slotCount < 0 || slotCount > 10000) throw new Error(`Invalid texture-slot count ${slotCount}`);
  for (let slot = 0; slot < slotCount; slot += 1) {
    reader.string();
    reader.i32();
    reader.string();
  }
  materialNames.push(`${name} [${shader}]`);
}

const glassCandidates = materialNames.filter((entry) => {
  const lower = entry.toLowerCase().replace(/[^a-z0-9]+/g, '_');
  const strong = ['window', 'windscreen', 'windshield', 'sideglass', 'side_glass', 'front_glass', 'rear_glass', 'glass_front', 'glass_rear', 'glass_int', 'glass_ext', 'cockpit_glass'];
  if (strong.some((token) => lower.includes(token))) return true;
  if (!lower.includes('glass')) return false;
  const reject = ['headlight', 'taillight', 'tail_light', 'brake_light', 'indicator', 'blinker', 'turn_light', 'reverse_light', 'lamp', 'reflector', 'glass_red', 'glass_orange'];
  return !reject.some((token) => lower.includes(token));
});

if (glassCandidates.length === 0) {
  throw new Error(`No cabin-glass material candidates found. Materials: ${materialNames.join(', ')}`);
}

console.log(`FULL QUALITY M5 verified: ${kn5.byteLength.toLocaleString()} bytes, KN5 v${version}, ${textureCount} textures, ${materialCount} materials.`);
console.log(`SHA-256: ${sha256}`);
console.log(`Cabin-glass material candidates (${glassCandidates.length}): ${glassCandidates.join(', ')}`);
