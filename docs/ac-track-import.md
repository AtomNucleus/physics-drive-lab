# Assetto Corsa track import pipeline

The simulator can ingest permitted Assetto Corsa KN5 track assets without Blender. Conversion happens offline; the browser never parses KN5 files.

## Pipeline

```text
track.kn5 + data/surfaces.ini
             or
models_layout.ini + multiple KN5s + layout/data/surfaces.ini
          |
          v
npm run import:ac-track
          |
          v
track.json
visual.bin
collision.bin
textures/
```

`visual.bin` contains packed render geometry. `collision.bin` contains only physical surface triangles used by the tire/suspension sampler. `track.json` binds the geometry, materials, surfaces, spawn data, bounds and source metadata together.

## Import a single-KN5 track

```bash
npm run import:ac-track -- /path/to/track.kn5 \
  --out ./public/tracks/mountain-test \
  --id mountain-test \
  --name "Mountain Test"
```

The importer automatically looks for `data/surfaces.ini` next to the KN5. Pass `--surfaces /path/to/surfaces.ini` when the layout differs.

## Import a normal Assetto Corsa layout

Many real AC tracks use `models_<layout>.ini` to combine scenery, pits and a dedicated physics KN5. Point the importer at that layout file instead of guessing one KN5:

```bash
npm run import:ac-track -- /path/to/models_mountain_downhill.ini \
  --out ./public/tracks/mountain-downhill \
  --id mountain-downhill \
  --name "Mountain Downhill"
```

For layout INIs the importer:

- reads every `[MODEL_n] FILE=...` entry;
- automatically detects a KN5 whose filename contains `physics` or `collision` and uses it as the physical source;
- merges the other KN5s into one render package while remapping material IDs and de-duplicating identical embedded textures;
- looks for `<layout>/data/surfaces.ini` and then the normal `data/surfaces.ini` fallback;
- finds the KN5 dummy `AC_START_0` and uses its X/Z/yaw as the default spawn;
- refuses non-zero `MODEL_n POSITION` / `ROTATION` transforms until explicit support exists rather than silently generating misaligned geometry.

You can override collision or spawn explicitly:

```bash
npm run import:ac-track -- /path/to/models_layout.ini \
  --collision-kn5 /path/to/physics.kn5 \
  --spawn-x 100 --spawn-z 200 --spawn-yaw 1.57
```

## Large-track browser budgets

A full AC layout can contain hundreds of megabytes of textures. Use repeatable visual filters to make a lighter browser package while retaining the full dedicated physics KN5:

```bash
npm run import:ac-track -- /path/to/models_layout.ini \
  --include-model main \
  --include-model pits \
  --include-model background \
  --out ./public/tracks/mountain-lite
```

Or exclude especially large scenery groups:

```bash
npm run import:ac-track -- /path/to/models_layout.ini \
  --exclude-model crowd \
  --exclude-model buildings
```

`--include-model` and `--exclude-model` affect visual KN5s only. The selected physics/collision KN5 remains intact.

If `surfaces.ini` is unavailable, the importer uses conservative mesh-name heuristics for common road, kerb, grass, gravel, dirt, sand and concrete names. It aborts instead of inventing an invisible road when no physical surface can be identified.

## Runtime loading

```ts
import { loadTrackPackage } from './tracks/TrackPackageLoader';

const track = await loadTrackPackage('/tracks/mountain-test/track.json');
scene.add(track.visualRoot);
physicsEngine.setSurfaceProvider(track.surfaceProvider);
physicsEngine.reset(
  track.manifest.spawnPoints[0].x,
  track.manifest.spawnPoints[0].z,
  track.manifest.spawnPoints[0].yaw
);
```

The flat proving ground remains the default surface provider and therefore stays available for controlled vehicle-physics regression testing.

## Physics behavior

Imported driveable geometry is sampled from the actual source triangles rather than a height approximation. The provider returns:

- exact interpolated road elevation at the wheel X/Z position;
- source-triangle surface normal for slope and camber;
- friction, rolling resistance, wetness and surface type;
- no fallback floor outside imported physical geometry.

An optional reference height can disambiguate stacked road layers such as bridges. A normal mountain pass typically does not need this feature, but the package format supports it.

Assetto Corsa's `IS_VALID_TRACK=0` flag is preserved as metadata. It does **not** disable physical contact: grass, gravel and other invalid-for-lap surfaces must still support the vehicle. The flag can be used later for timing and course-cut validation.

Dedicated physics KN5s sometimes contain retaining-wall faces or mesh sidewalls under ROAD-like names. Wheel-support extraction therefore removes obvious wall/barrier/guardrail meshes and rejects implausibly steep faces using surface-specific slope limits. Those obstacle faces should later feed the chassis-collision system rather than the tire height sampler.

## Current scope

This slice covers tire/suspension road contact and visual scenery loading. Chassis collision against guardrails, concrete walls, rock faces and other barriers is intentionally a separate next step so road-contact correctness can be validated independently.

For maximum mountain-road fidelity, tire longitudinal/lateral forces should also be resolved in the road tangent plane rather than only the chassis X/Z plane. The imported triangle normal is already available for that next physics step.

## Asset rights

Only convert or redistribute tracks and textures when their license or creator permission allows it. The importer is an engine capability; it does not grant rights to third-party Assetto Corsa mods.
