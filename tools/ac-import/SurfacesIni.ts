import { readFile } from 'node:fs/promises';

export interface AcSurfaceDefinition {
  section: string;
  key: string;
  friction: number;
  damping: number;
  isValidTrack: boolean;
  raw: Record<string, string>;
}

export function parseSurfacesIni(text: string): AcSurfaceDefinition[] {
  const sections: { name: string; values: Record<string, string> }[] = [];
  let current: { name: string; values: Record<string, string> } | null = null;

  for (const rawLine of text.split(/\r?\n/)) {
    const line = rawLine.replace(/[;#].*$/, '').trim();
    if (!line) continue;
    const sectionMatch = /^\[([^\]]+)\]$/.exec(line);
    if (sectionMatch) { current = { name: sectionMatch[1], values: {} }; sections.push(current); continue; }
    const equals = line.indexOf('=');
    if (equals < 0 || !current) continue;
    current.values[line.slice(0, equals).trim().toUpperCase()] = line.slice(equals + 1).trim();
  }

  return sections.filter((section) => section.name.toUpperCase().startsWith('SURFACE_')).map((section) => ({
    section: section.name,
    key: (section.values.KEY || section.name).toUpperCase(),
    friction: finiteNumber(section.values.FRICTION, 1),
    damping: finiteNumber(section.values.DAMPING, 0),
    isValidTrack: finiteNumber(section.values.IS_VALID_TRACK, 1) !== 0,
    raw: section.values,
  }));
}

function finiteNumber(value: string | undefined, fallback: number): number {
  const parsed = value === undefined ? Number.NaN : Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

export async function readSurfacesIni(path: string): Promise<AcSurfaceDefinition[]> {
  return parseSurfacesIni(await readFile(path, 'utf8'));
}
