# 2025 BMW M5 Validation Suite

The M5 validation suite is the quantitative physics-regression loop for the G90 simulation.

Its job is not to make the vehicle look realistic. Its job is to measure whether the normal simulation produces defensible vehicle-dynamics results under repeatable inputs.

## Rule: validation never drives the answer

Validation code may prescribe:

- initial position, heading and speed
- throttle, brake and steering commands
- road friction, grade, wetness and bump geometry
- tire-state parameters already supported by the model
- ABS/TCS mode selections already supported by the model

Validation code must never prescribe:

- chassis position or yaw during a maneuver
- tire force
- hidden grip
- yaw damping
- body-roll animation
- acceleration/braking multipliers
- test-specific steering corrections inside the physics model
- invisible stability assistance

The closed-loop skidpad driver adjusts only the driver's throttle, brake and steering command. It does not alter the vehicle state.

## Run the full suite

```bash
npm run test:m5-validation
```

This runs the normal quantitative suite and exits non-zero only for blocking physics invariants/numerical failures. A mismatch with an external performance reference is still recorded as `FAIL`, but does not prevent the framework from generating the evidence needed to diagnose it.

To make every `FAIL` return a non-zero process status:

```bash
npm run test:m5-validation:strict
```

## List or run individual tests

```bash
npm run test:m5-validation:list
npx tsx src/physics/validation/M5ValidationSuite.ts --test=acceleration
npx tsx src/physics/validation/M5ValidationSuite.ts --test=skidpad,step-steer
```

Available test IDs:

- `determinism`
- `static-loads`
- `mass-properties-moments`
- `acceleration`
- `braking`
- `skidpad`
- `step-steer`
- `rapid-reversal`
- `slalom`
- `bump-response`
- `lift-throttle`
- `energy-sanity`

## Artifacts

By default the suite writes to:

```text
artifacts/m5-validation/
```

Key outputs:

- `m5-validation-report.json` — machine-readable complete report
- `m5-validation-report.md` — human-readable report
- `m5-validation-metrics.csv` — flattened metric/status table
- `telemetry/*.csv` — per-test 120 Hz telemetry
- `skidpad-sweep.csv` — steady-state radius/G/load/slip sweep
- `acceleration-speed.svg`
- `acceleration-loads.svg`
- `braking-100kmh.svg`
- `skidpad-steering-vs-lateral-g.svg`
- `step-steer-80kmh.svg`
- `bump-response.svg`

Choose another directory with:

```bash
npx tsx src/physics/validation/M5ValidationSuite.ts --artifacts=artifacts/my-run
```

## Before/after regression comparison

Run a new suite against a previously saved JSON report:

```bash
npx tsx src/physics/validation/M5ValidationSuite.ts \
  --baseline=artifacts/baselines/m5-validation-report.json
```

The new report includes percentage deltas for matching numeric metrics. The baseline is descriptive evidence, not permission to preserve wrong physics. A known-good internal regression can protect behavior while real-world references remain unavailable, but an external mismatch should still be investigated.

## What is measured now

### Deterministic harness

- fixed 120 Hz timestep
- automatic reset and settling
- identical scripted input replay
- configurable friction, grade, wetness, split-μ and bump profiles
- tire temperature/pressure/wear hooks
- ABS/TCS mode hooks

### Vehicle and chassis

- position and world/body velocity
- body-local acceleration
- lateral/longitudinal/vertical G
- yaw/pitch/roll
- yaw/pitch/roll rate and angular acceleration
- sideslip
- gear/RPM and assist activity

### Controls / steering

- throttle, brake and normalized steering command
- all four road-wheel angles
- steering-wheel angle when a physical steering model exposes it
- rack position when a physical steering model exposes it

PR #27 does not yet contain the separate physical steering-rack PR. On that base, steering-wheel angle is explicitly labeled `derived-overall-ratio` using BMW's published 14.2:1 overall ratio, and rack position remains null. The derived angle must not be presented as simulated rack telemetry.

### Four wheels (FL, FR, RL, RR)

- Fz / Fx / Fy
- slip angle and slip ratio
- wheel angular velocity and wheel speed
- suspension displacement, velocity and compression
- spring, damper, bump-stop and anti-roll-bar forces
- camber and toe when available
- road-wheel steer angle
- contact state
- aligning moment / pneumatic trail
- temperature / pressure / wear
- local surface friction
- grip utilization

## Current engineering tests

### Static axle loading

Checks:

- `ΣFz ≈ mg`
- configured front/rear distribution
- left/right static symmetry
- residual speed at rest

### CG / inertia / moment closure

Checks the analytical rigid-body invariant:

```text
τ = r × F
α = I⁻¹(τ − ω × Iω)
```

and also reconstructs live tire/contact yaw moment during an 80 km/h steering transient. This is intended to catch a bad CG origin, force arm, coordinate transform or inertia tensor before anyone tries to tune grip.

### Acceleration

Measures:

- 0–30 km/h
- 0–50 km/h
- true-start 0–60 mph
- 0–100 km/h
- 0–120 km/h
- quarter mile and trap speed
- peak longitudinal G
- wheel slip
- pitch and tire loads

Launch uses the normal launch-control, powertrain, differential, tire and TCS paths.

### Braking

Measures:

- 100–0 km/h
- 70–0 mph
- 100–0 mph
- stopping time
- peak/average deceleration
- ABS activity
- wheel slip/load transfer
- pitch

### Skidpad / understeer / roll

Runs 20 m, 30 m, 45.72 m, 50 m and 100 m radii. The 45.72 m radius corresponds to Car and Driver's 300-ft diameter skidpad.

Measures:

- steady lateral G
- actual radius
- speed
- road-wheel steering demand
- estimated/simulated steering-wheel demand
- yaw rate and sideslip
- roll
- individual Fz/Fy/slip angle
- road-wheel understeer gradient
- roll gradient

Only peak skidpad G currently has a direct real-world G90 reference in the suite. Understeer and roll gradient remain `REFERENCE DATA NEEDED`.

### Step steer

Runs at 30, 50, 80 and 100 km/h and measures:

- steering response onset
- tire-slip onset
- yaw-response delay
- 10–90% yaw rise time
- yaw overshoot
- settling time after release
- yaw-rate gain
- tire loads / roll / lateral G

The required causal order is steering → tire slip/force → chassis yaw/load transfer/roll, not instant body rotation.

### Rapid reversal and slalom

Exercises load reversal, suspension travel, tire-force sign changes and numerical stability without hidden ESC or yaw damping.

### Bump / unsprung response

Uses a smooth raised-cosine road bump and supports left-wheel and full-width profiles. Separate starting positions allow a front or rear axle encounter without forcing the other axle onto the bump first.

Estimates:

- wheel-hop frequency
- body-heave frequency
- wheel-response time
- chassis-response time

The qualitative invariant is road → wheel/unsprung mass → spring/damper → chassis.

### Lift-off / throttle-on

Records natural yaw/load/slip redistribution on lift and combined-slip behavior when power is added in a corner. No lift-off oversteer is forced.

### Energy and low-speed sanity

Guards against:

- no-throttle coasting gaining kinetic energy
- the historical low-speed turn acceleration bug
- steering at rest creating spontaneous yaw or vehicle motion
- NaN/Infinity
- force on an airborne tire
- negative Fz
- extreme suspension travel/velocity
- unbounded angular velocity

## Reference hierarchy

Reference data lives in `src/physics/validation/M5ReferenceData.ts`, separate from test logic.

The report distinguishes:

- `hard` — direct external measurement/specification
- `engineering-plausibility` — derived or literature-supported expectation
- `internal-regression` — deterministic behavior protected while external data is missing

If a direct value is unavailable, the result must say `NO REFERENCE DATA` / `REFERENCE DATA NEEDED`.

Current external anchors are BMW Group PressClub and Car and Driver instrumented results. Missing G90 data includes understeer gradient, roll gradient, step-steer/yaw response, dynamic wheel loads, suspension travel versus G, pitch gradient, wheel-hop/heave frequencies and lift/throttle transient telemetry.

## Development loop

Use this sequence for physics work:

```text
real-world measurement
→ simulation measurement
→ error
→ physical diagnosis
→ physics correction
→ regression comparison
```

Do not change a coefficient simply because a metric is red. First inspect the causal chain that produced it.
