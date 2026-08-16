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
}

export interface SuspensionState {
  displacement: number; // + bump/compression, - droop (m)
  velocity: number; // displacement velocity (m/s)
  compressionRatio: number;
  hubTravelM: number;
  bumpStopEngaged: boolean;
  forceNorm: number; // per-wheel Fz delivered to tire (N)
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
  forceNorm: 0,
  dynamicCamberDeg: 0,
  isAirborne: true,
  contactPointWorld: PhysicsMath.vec3(),
  tireCompressionM: 0,
});

/**
 * Four independent spring/damper corners with axle anti-roll coupling.
 * There is intentionally no analytic "weight transfer" shortcut here:
 * chassis pitch/roll moves the hardpoints, which changes spring compression,
 * which changes each tire's vertical load over time.
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

    for (let i = 0; i < 4; i++) {
      const cfg = configs[i];
      const prev = this.states[i];
      const hpBody = hardpointsBody[i];
      const hpWorldOffset = PhysicsMath.quatRotateVec3(bodyOrientation, hpBody);
      const hpWorld = PhysicsMath.vec3Add(bodyPosition, hpWorldOffset);
      const surface = sampleSurface(hpWorld.x, hpWorld.z);

      // Suspension axis is body-down. For the proving ground's modest grades,
      // solving against ground elevation is deterministic and keeps all four
      // corners responsive to chassis pitch and roll.
      // Vehicle hardpoints are plan-view chassis anchors (their local Y is 0).
      // Offset the virtual strut pickup downward by available droop so the
      // configured rest length represents the normal installed ride position.
      const strutPickupOffset = Math.max(0, cfg.maxDroop);
      const geometricLength = hpWorld.y - strutPickupOffset - surface.elevation - wheelRadius;
      const desiredDisplacement = cfg.restLength - geometricLength;

      const minDisplacement = -Math.max(0, cfg.maxDroop);
      const maxDisplacement = Math.max(0.001, cfg.maxBump);
      const displacement = PhysicsMath.clamp(desiredDisplacement, minDisplacement, maxDisplacement);
      const velocity = (displacement - prev.displacement) / dt;

      newDisplacements[i] = displacement;
      newVelocities[i] = velocity;

      // If the road is farther away than full droop can reach, the wheel is airborne.
      const canReachGround = desiredDisplacement >= minDisplacement - 1e-4;
      airborne[i] = !canReachGround;

      const currentLength = cfg.restLength - displacement;
      const wheelBottomY = hpWorld.y - strutPickupOffset - currentLength - wheelRadius;
      const tirePenetration = Math.max(0, surface.elevation - wheelBottomY);
      tireCompression[i] = tirePenetration;

      // Installed coilovers carry preload at nominal ride height. Treat the
      // configured rest position as the installed position, with preload that
      // fades to zero near full droop. This prevents a reset from free-falling
      // through a large chunk of suspension travel before the springs wake up.
      const preloadTravel = Math.max(0, cfg.maxDroop) * 0.78;
      let springForce = Math.max(0, displacement + preloadTravel) * Math.max(0, cfg.springStiffness);

      // Low-speed damping controls chassis attitude; high-speed damping lets
      // sharp bumps move without making the whole car artificially rigid.
      const absVel = Math.abs(velocity);
      let dampingCoeff: number;
      if (velocity < 0) {
        dampingCoeff = cfg.dampingRebound;
      } else {
        const blend = PhysicsMath.clamp((absVel - 0.12) / 0.65, 0, 1);
        dampingCoeff = PhysicsMath.lerp(cfg.dampingLowSpeed, cfg.dampingHighSpeed, blend);
      }
      const damperForce = dampingCoeff * velocity;

      const bumpThresholdDisplacement = maxDisplacement * PhysicsMath.clamp(cfg.bumpStopThreshold, 0, 1);
      let bumpStopForce = 0;
      if (displacement > bumpThresholdDisplacement) {
        const bumpTravel = displacement - bumpThresholdDisplacement;
        const bumpRange = Math.max(0.005, maxDisplacement - bumpThresholdDisplacement);
        const progress = PhysicsMath.clamp(bumpTravel / bumpRange, 0, 2);
        bumpStopForce = cfg.bumpStopStiffness * bumpTravel * (1 + 2.5 * progress * progress);
        bumpStops[i] = true;
      }

      // Tire radial compliance only becomes active after suspension travel is
      // geometrically exhausted. It prevents infinite ground penetration and
      // gives violent compressions a second spring stage.
      const tireForce = tirePenetration * Math.max(0, tireVerticalStiffness);

      rawForces[i] = airborne[i] ? 0 : Math.max(0, springForce + damperForce + bumpStopForce + tireForce);
      contacts[i] = PhysicsMath.vec3(hpWorld.x, surface.elevation, hpWorld.z);
    }

    // Anti-roll bars redistribute load across each axle. This is exactly the
    // behavior that makes the outside tire gain Fz while the inside tire sheds it.
    const applyAxleBar = (left: number, right: number, stiffness: number) => {
      const delta = newDisplacements[left] - newDisplacements[right];
      const transfer = stiffness * delta;
      rawForces[left] += transfer;
      rawForces[right] -= transfer;
    };
    applyAxleBar(0, 1, rollStiffnessFront);
    applyAxleBar(2, 3, rollStiffnessRear);

    // Small diagonal coupling makes a one-wheel kerb strike communicate through
    // the chassis without replacing the independent suspension motion.
    const cross = PhysicsMath.clamp(antiRollCrossCoupling, 0, 1);
    if (cross > 0) {
      const diagonalDelta = (newDisplacements[0] + newDisplacements[3]) - (newDisplacements[1] + newDisplacements[2]);
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
        forceNorm: fz,
        // Bump generates more negative camber on a conventional performance suspension.
        dynamicCamberDeg: cfg.staticCamberDeg - cfg.camberGainDegPerMeter * Math.max(0, displacement),
        isAirborne: airborne[i],
        contactPointWorld: contacts[i],
        tireCompressionM: tireCompression[i],
      };
    }
  }
}
