import { Vec3, Quat, PhysicsMath } from './math/PhysicsMath';

export interface SuspensionCornerConfig {
  restLength: number;
  springStiffness: number;
  dampingLowSpeed: number;
  dampingHighSpeed: number;
  dampingRebound: number;
  bumpStopStiffness: number;
  bumpStopThreshold: number;
  maxDroop: number;
  maxBump: number;
  staticCamberDeg: number;
  camberGainDegPerMeter: number;
  antiDiveSquatRatio: number;
  springProgressionRatio?: number;
  dampingReboundHighSpeed?: number;
  damperKneeVelocity?: number;
  damperHighSpeedVelocity?: number;
  bumpStopProgression?: number;
  bumpStopExponent?: number;
}

export interface SuspensionState {
  displacement: number;
  velocity: number;
  compressionRatio: number;
  hubTravelM: number;
  bumpStopEngaged: boolean;
  atCompressionLimit: boolean;
  atReboundLimit: boolean;
  forceNorm: number;
  springForceN: number;
  damperForceN: number;
  bumpStopForceN: number;
  hardStopForceN: number;
  dynamicCamberDeg: number;
  isAirborne: boolean;
  contactPointWorld: Vec3;
  tireCompressionM: number;
}

type SurfaceLike = {
  elevation: number;
  normal: Vec3;
};

const makeState = (): SuspensionState => ({
  displacement: 0,
  velocity: 0,
  compressionRatio: 0,
  hubTravelM: 0,
  bumpStopEngaged: false,
  atCompressionLimit: false,
  atReboundLimit: false,
  forceNorm: 0,
  springForceN: 0,
  damperForceN: 0,
  bumpStopForceN: 0,
  hardStopForceN: 0,
  dynamicCamberDeg: 0,
  isAirborne: true,
  contactPointWorld: PhysicsMath.vec3(),
  tireCompressionM: 0,
});

/**
 * Incremental coil-spring force about the installed/preloaded ride position.
 * Compression is progressive; droop remains linear until spring load reaches zero.
 */
export function progressiveSpringIncrement(
  displacement: number,
  baseRate: number,
  maxBump: number,
  progressionRatio: number = 0.65
): number {
  const k = Math.max(0, baseRate);
  if (displacement <= 0 || k === 0) return k * displacement;

  const travel = Math.max(0.001, maxBump);
  const progression = Math.max(0, progressionRatio);
  const x = displacement;

  // Integrate k(x) = k0 * (1 + progression * (x / travel)^2).
  return k * x + (k * progression * x * x * x) / (3 * travel * travel);
}

/**
 * 2-way digressive damper: bump/compression and rebound have independent
 * low- and high-shaft-speed valving. Positive velocity is compression.
 */
export function damperForceForVelocity(
  velocity: number,
  compressionLowSpeed: number,
  compressionHighSpeed: number,
  reboundLowSpeed: number,
  reboundHighSpeed: number,
  kneeVelocity: number = 0.12,
  highSpeedVelocity: number = 0.77
): number {
  if (!Number.isFinite(velocity) || velocity === 0) return 0;

  const absVelocity = Math.abs(velocity);
  const knee = Math.max(0, kneeVelocity);
  const span = Math.max(0.01, highSpeedVelocity - knee);
  const linearBlend = PhysicsMath.clamp((absVelocity - knee) / span, 0, 1);
  const blend = linearBlend * linearBlend * (3 - 2 * linearBlend);

  const low = velocity > 0
    ? Math.max(0, compressionLowSpeed)
    : Math.max(0, reboundLowSpeed);
  const high = velocity > 0
    ? Math.max(0, compressionHighSpeed)
    : Math.max(0, reboundHighSpeed);

  return PhysicsMath.lerp(low, high, blend) * velocity;
}

/** Progressive elastomer bump-stop load once the configured jounce threshold is reached. */
export function bumpStopForceForDisplacement(
  displacement: number,
  maxBump: number,
  thresholdRatio: number,
  stiffness: number,
  progression: number = 4.0,
  exponent: number = 2.2
): number {
  const travel = Math.max(0.001, maxBump);
  const threshold = travel * PhysicsMath.clamp(thresholdRatio, 0, 0.98);
  if (displacement <= threshold) return 0;

  const bumpRange = Math.max(0.003, travel - threshold);
  const bumpTravel = displacement - threshold;
  const progress = Math.max(0, bumpTravel / bumpRange);

  return Math.max(0, stiffness) * bumpTravel *
    (1 + Math.max(0, progression) * Math.pow(progress, Math.max(1, exponent)));
}

/**
 * Four independent spring/damper corners with axle anti-roll coupling.
 * Chassis pitch/roll moves the hardpoints, which changes each corner's jounce,
 * spring/damper load, and therefore the actual rigid-body moments. No visual
 * body-roll force is generated here.
 */
export class SuspensionSystem {
  public states: [SuspensionState, SuspensionState, SuspensionState, SuspensionState] = [
    makeState(), makeState(), makeState(), makeState(),
  ];

  public reset() {
    this.states = [makeState(), makeState(), makeState(), makeState()];
  }

  public update(
    hardpointsBody: [Vec3, Vec3, Vec3, Vec3],
    bodyPosition: Vec3,
    bodyOrientation: Quat,
    bodyVelocityWorld: Vec3,
    bodyAngularVelocityWorld: Vec3,
    sampleSurface: (x: number, z: number) => SurfaceLike,
    configs: [SuspensionCornerConfig, SuspensionCornerConfig, SuspensionCornerConfig, SuspensionCornerConfig],
    rollStiffnessFront: number,
    rollStiffnessRear: number,
    antiRollCrossCoupling: number,
    wheelRadius: number,
    tireVerticalStiffness: number,
    dt: number
  ) {
    if (dt <= 0) return;

    const rawForces = [0, 0, 0, 0];
    const newDisplacements = [0, 0, 0, 0];
    const newVelocities = [0, 0, 0, 0];
    const airborne = [false, false, false, false];
    const contacts: Vec3[] = new Array(4);
    const tireCompression = [0, 0, 0, 0];
    const bumpStops = [false, false, false, false];
    const compressionLimits = [false, false, false, false];
    const reboundLimits = [false, false, false, false];
    const springForces = [0, 0, 0, 0];
    const damperForces = [0, 0, 0, 0];
    const bumpStopForces = [0, 0, 0, 0];
    const hardStopForces = [0, 0, 0, 0];

    for (let i = 0; i < 4; i++) {
      const cfg = configs[i];
      const prev = this.states[i];
      const hpBody = hardpointsBody[i];
      const hpWorldOffset = PhysicsMath.quatRotateVec3(bodyOrientation, hpBody);
      const hpWorld = PhysicsMath.vec3Add(bodyPosition, hpWorldOffset);
      const surface = sampleSurface(hpWorld.x, hpWorld.z);

      // The configured wheel travel is a hard physical constraint. The virtual
      // pickup offset preserves the existing installed ride-height convention.
      const strutPickupOffset = Math.max(0, cfg.maxDroop);
      const geometricLength = hpWorld.y - strutPickupOffset - surface.elevation - wheelRadius;
      const desiredDisplacement = cfg.restLength - geometricLength;

      const minDisplacement = -Math.max(0, cfg.maxDroop);
      const maxDisplacement = Math.max(0.001, cfg.maxBump);
      const displacement = PhysicsMath.clamp(desiredDisplacement, minDisplacement, maxDisplacement);
      const velocity = (displacement - prev.displacement) / dt;

      newDisplacements[i] = displacement;
      newVelocities[i] = velocity;
      compressionLimits[i] = desiredDisplacement >= maxDisplacement - 1e-5;
      reboundLimits[i] = desiredDisplacement <= minDisplacement + 1e-5;

      // Once full droop cannot reach the road, that corner contributes no normal load.
      const canReachGround = desiredDisplacement >= minDisplacement - 1e-4;
      airborne[i] = !canReachGround;

      const currentLength = cfg.restLength - displacement;
      const wheelBottomY = hpWorld.y - strutPickupOffset - currentLength - wheelRadius;
      const tirePenetration = Math.max(0, surface.elevation - wheelBottomY);
      tireCompression[i] = tirePenetration;

      // Installed spring preload carries static chassis load at nominal ride height.
      const preloadTravel = Math.max(0, cfg.maxDroop) * 0.78;
      const preloadForce = preloadTravel * Math.max(0, cfg.springStiffness);
      const springForce = Math.max(
        0,
        preloadForce + progressiveSpringIncrement(
          displacement,
          cfg.springStiffness,
          maxDisplacement,
          cfg.springProgressionRatio ?? 0.65
        )
      );
      springForces[i] = springForce;

      // Compression and rebound use different valving on both sides of the shaft-speed knee.
      const reboundHighSpeed = cfg.dampingReboundHighSpeed ??
        Math.max(cfg.dampingHighSpeed * 1.10, cfg.dampingRebound * 0.62);
      const damperForce = damperForceForVelocity(
        velocity,
        cfg.dampingLowSpeed,
        cfg.dampingHighSpeed,
        cfg.dampingRebound,
        reboundHighSpeed,
        cfg.damperKneeVelocity ?? 0.12,
        cfg.damperHighSpeedVelocity ?? 0.77
      );
      damperForces[i] = damperForce;

      const bumpStopForce = bumpStopForceForDisplacement(
        displacement,
        maxDisplacement,
        cfg.bumpStopThreshold,
        cfg.bumpStopStiffness,
        cfg.bumpStopProgression ?? 4.0,
        cfg.bumpStopExponent ?? 2.2
      );
      bumpStopForces[i] = bumpStopForce;
      bumpStops[i] = bumpStopForce > 0;

      // Past full jounce, suspension travel cannot continue. A very stiff hard-stop
      // contact plus tire radial compliance prevents the chassis from treating the
      // tire as extra suspension travel.
      const overTravel = Math.max(0, desiredDisplacement - maxDisplacement);
      const hardStopRate = Math.max(
        Math.max(0, cfg.bumpStopStiffness) * 8,
        Math.max(0, tireVerticalStiffness) * 1.5
      );
      const hardStopForce = overTravel * hardStopRate;
      hardStopForces[i] = hardStopForce;

      const tireForce = tirePenetration * Math.max(0, tireVerticalStiffness);

      rawForces[i] = airborne[i]
        ? 0
        : Math.max(0, springForce + damperForce + bumpStopForce + hardStopForce + tireForce);
      contacts[i] = PhysicsMath.vec3(hpWorld.x, surface.elevation, hpWorld.z);
    }

    // Anti-roll bars transfer load across an axle; they do not animate the body.
    const applyAxleBar = (left: number, right: number, stiffness: number) => {
      const delta = newDisplacements[left] - newDisplacements[right];
      const transfer = stiffness * delta;
      rawForces[left] += transfer;
      rawForces[right] -= transfer;
    };
    applyAxleBar(0, 1, rollStiffnessFront);
    applyAxleBar(2, 3, rollStiffnessRear);

    const cross = PhysicsMath.clamp(antiRollCrossCoupling, 0, 1);
    if (cross > 0) {
      const diagonalDelta =
        (newDisplacements[0] + newDisplacements[3]) -
        (newDisplacements[1] + newDisplacements[2]);
      const averageBar = 0.25 * (rollStiffnessFront + rollStiffnessRear);
      const crossTransfer = diagonalDelta * averageBar * cross * 0.20;
      rawForces[0] += crossTransfer;
      rawForces[3] += crossTransfer;
      rawForces[1] -= crossTransfer;
      rawForces[2] -= crossTransfer;
    }

    for (let i = 0; i < 4; i++) {
      const cfg = configs[i];
      const displacement = newDisplacements[i];
      const fz = airborne[i] ? 0 : Math.max(0, rawForces[i]);

      this.states[i] = {
        displacement,
        velocity: newVelocities[i],
        compressionRatio: PhysicsMath.clamp(displacement / Math.max(0.001, cfg.maxBump), 0, 1),
        hubTravelM: displacement,
        bumpStopEngaged: bumpStops[i],
        atCompressionLimit: compressionLimits[i],
        atReboundLimit: reboundLimits[i],
        forceNorm: fz,
        springForceN: springForces[i],
        damperForceN: damperForces[i],
        bumpStopForceN: bumpStopForces[i],
        hardStopForceN: hardStopForces[i],
        dynamicCamberDeg: cfg.staticCamberDeg - cfg.camberGainDegPerMeter * Math.max(0, displacement),
        isAirborne: airborne[i],
        contactPointWorld: contacts[i],
        tireCompressionM: tireCompression[i],
      };
    }
  }
}
