# Assetto Corsa track import pipeline

The simulator can ingest permitted Assetto Corsa KN5 track assets without Blender. Conversion happens offline; the browser never parses KN5 files.

## Pipeline

```text
track.kn5 + data/surfaces.ini
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

## Import a track

```bash
npm run import:ac-track -- /path/to/track.kn5 \
  --out ./public/tracks/mountain-test \
  --id mountain-test \
  --name "Mountain Test" \
  --spawn-x 0 \
  --spawn-z 0 \
  --spawn-yaw 0
```

The importer automatically looks for `data/surfaces.ini` next to the KN5. Pass `--surfaces /path/to/surfaces.ini` when the layout differs.

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

## Current scope

This first slice covers tire/suspension road contact and visual scenery loading. Chassis collision against guardrails, concrete walls, rock faces and other barriers is intentionally a separate next step so road-contact correctness can be validated independently.

## Asset rights

Only convert or redistribute tracks and textures when their license or creator permission allows it. The importer is an engine capability; it does not grant rights to third-party Assetto Corsa mods.
