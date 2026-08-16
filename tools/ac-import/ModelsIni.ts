import { readFile } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';

export interface AcLayoutModel {
  section: string;
  file: string;
  path: string;
  position: [number, number, number];
  rotation: [number, number, number];
}

function parseVec3(value: string | undefined): [number, number, number] {
  const parts = (value ?? '0,0,0').split(',').map((part) => Number(part.trim()));
  return [
    Number.isFinite(parts[0]) ? parts[0] : 0,
    Number.isFinite(parts[1]) ? parts[1] : 0,
    Number.isFinite(parts[2]) ? parts[2] : 0,
  ];
}

export function parseModelsIni(text: string, baseDir: string): AcLayoutModel[] {
  const sections: { name: string; values: Record<string, string> }[] = [];
  let current: { name: string; values: Record<string, string> } | null = null;

  for (const rawLine of text.split(/\r?\n/)) {
    const line = rawLine.replace(/[;#].*$/, '').trim();
    if (!line) continue;
    const sectionMatch = /^\[([^\]]+)\]$/.exec(line);
    if (sectionMatch) {
      current = { name: sectionMatch[1], values: {} };
      sections.push(current);
      continue;
    }
    const equals = line.indexOf('=');
    if (equals < 0 || !current) continue;
    current.values[line.slice(0, equals).trim().toUpperCase()] = line.slice(equals + 1).trim();
  }

  return sections
    .filter((section) => section.name.toUpperCase().startsWith('MODEL_') && section.values.FILE)
    .map((section) => ({
      section: section.name,
      file: section.values.FILE,
      path: resolve(baseDir, section.values.FILE),
      position: parseVec3(section.values.POSITION),
      rotation: parseVec3(section.values.ROTATION),
    }));
}

export async function readModelsIni(path: string): Promise<AcLayoutModel[]> {
  return parseModelsIni(await readFile(path, 'utf8'), dirname(path));
}

export function hasExternalModelTransform(model: AcLayoutModel): boolean {
  return [...model.position, ...model.rotation].some((value) => Math.abs(value) > 1e-6);
}
