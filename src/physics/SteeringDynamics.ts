import type { Vehicle } from './Vehicle';
import type { WheelKinematicPose } from './SuspensionKinematics';
import { PhysicsMath } from './math/PhysicsMath';

export interface SteeringTorqueBreakdown {
  tireSelfAligningRoadNm: number;
  casterMechanicalTrailRoadNm: number;
  steeringDampingRoadNm: number;
  steeringFrictionRoadNm: number;
  steeringInertiaRoadNm: number;
  epsAssistSteeringWheelNm: number;
  driverSteeringWheelNm: number;
  steeringStopRoadNm: number;
  netRackRoadNm: number;
  ffbReadySteeringWheelNm: number;
}

export interface SteeringDynamicsTelemetry {
  driverInput: number;
  targetSteeringWheelAngleRad: number;
  steeringWheelAngleRad: number;
  steeringWheelVelocityRadS: number;
  localSteeringRatio: number;
  rackDisplacementM: number;
  rackVelocityMS: number;
  rackCenterAngleRad: number;
  rackAngularVelocityRadS: number;
  rackAngularAccelerationRadS2: number;
  leftRoadWheelAngleRad: number;
  rightRoadWheelAngleRad: number;
  leftComplianceRad: number;
  rightComplianceRad: number;
  leftMechanicalTrailM: number;
  rightMechanicalTrailM: number;
  epsAssistGain: number;
  torques: SteeringTorqueBreakdown;
}

interface SteeringCalibration {
  wheelbaseM: number;
  frontTrackM: number;
  ackermannRatio: number;
  maxRoadWheelAngleRad: number;
  rackHalfTravelM: number;
  steeringRatioCenter: number;
  steeringRatioAtLock: number;
  rackEquivalentInertiaKgm2: number;
  rackDampingNmsPerRad: number;
  rackFrictionNm: number;
  maxRackAngularSpeedRadS: number;
  maxRackAngularAccelRadS2: number;
  driverTorsionStiffnessNmPerRad: number;
  driverTorsionDampingNmsPerRad: number;
  driverMaxTorqueNm: number;
  epsAssistParkingGain: number;
  epsAssistHighSpeedGain: number;
  epsAssistFadeSpeedMs: number;
  epsMaxAssistTorqueNm: number;
  stopStartFraction: number;
  stopStiffnessNmPerRad: number;
  stopDampingNmsPerRad: number;
  tieRodStiffnessNmPerRad: number;
  rackMountStiffnessNmPerRad: number;
  controlArmBushingStiffnessNmPerRad: number;
  tireCarcassSteerStiffnessNmPerRad: number;
  complianceDampingNmsPerRad: number;
  maxComplianceRad: number;
}

const ZERO_TORQUES = (): SteeringTorqueBreakdown => ({
  tireSelfAligningRoadNm: 0,
  casterMechanicalTrailRoadNm: 0,
  steeringDampingRoadNm: 0,
  steeringFrictionRoadNm: 0,
  steeringInertiaRoadNm: 0,
  epsAssistSteeringWheelNm: 0,
  driverSteeringWheelNm: 0,
  steeringStopRoadNm: 0,
  netRackRoadNm: 0,
  ffbReadySteeringWheelNm: 0,
});

/**
 * Physical steering-column/rack model for the front axle.
 *
 * The generalized coordinate is the center road-wheel steering angle. Driver input
 * is a steering-wheel target acting through column torsion; it is never assigned to
 * a road wheel directly. Column torque and EPS torque drive a rack with inertia and
 * damping. The rack then produces Ackermann left/right commands, while the previous
 * contact-patch step feeds self-aligning and geometry-derived steering-axis torque
 * back into the rack. This one fixed-step feedback delay is intentional and keeps
 * the 120 Hz solve explicit/stable without inventing a speed-based centering spring.
 */
export class PhysicalSteeringSystem {
  private readonly vehicle: Vehicle;
  private readonly getPoses: () => [WheelKinematicPose, WheelKinematicPose, WheelKinematicPose, WheelKinematicPose];
  private config: SteeringCalibration;

  private rackCenterAngleRad = 0;
  private rackAngularVelocityRadS = 0;
  private rackAngularAccelerationRadS2 = 0;
  private complianceComponents: [number[], number[]] = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
  ];

  public telemetry: SteeringDynamicsTelemetry;

  constructor(
    vehicle: Vehicle,
    getPoses: () => [WheelKinematicPose, WheelKinematicPose, WheelKinematicPose, WheelKinematicPose]
  ) {
    this.vehicle = vehicle;
    this.getPoses = getPoses;
    this.config = this.readCalibration();
    this.telemetry = this.makeTelemetry();
  }

  public reconfigure() {
    this.config = this.readCalibration();
    this.rackCenterAngleRad = PhysicsMath.clamp(
      this.rackCenterAngleRad,
      -this.config.maxRoadWheelAngleRad,
      this.config.maxRoadWheelAngleRad
    );
  }

  public reset() {
    this.rackCenterAngleRad = 0;
    this.rackAngularVelocityRadS = 0;
    this.rackAngularAccelerationRadS2 = 0;
    this.complianceComponents = [
      [0, 0, 0, 0],
      [0, 0, 0, 0],
    ];
    this.telemetry = this.makeTelemetry();
  }

  private readCalibration(): SteeringCalibration {
    const c = this.vehicle.config as any;
    return {
      wheelbaseM: Math.max(0.5, Number(c.wheelbase ?? 3.00482)),
      frontTrackM: Math.max(0.5, Number(c.steeringFrontTrackM ?? c.trackWidth ?? 1.68402)),
      ackermannRatio: PhysicsMath.clamp(Number(c.ackermannRatio ?? 0.90), 0, 1.15),
      maxRoadWheelAngleRad: Math.max(0.15, Number(c.maxSteerAngle ?? 0.58)),
      rackHalfTravelM: Math.max(0.02, Number(c.steeringRackHalfTravelM ?? 0.070)),
      steeringRatioCenter: Math.max(5, Number(c.steeringRatioCenter ?? 14.2)),
      steeringRatioAtLock: Math.max(5, Number(c.steeringRatioAtLock ?? 12.6)),
      rackEquivalentInertiaKgm2: Math.max(0.2, Number(c.steeringRackEquivalentInertiaKgm2 ?? 5.8)),
      rackDampingNmsPerRad: Math.max(0, Number(c.steeringRackDampingNmsPerRad ?? 46)),
      rackFrictionNm: Math.max(0, Number(c.steeringRackFrictionNm ?? 4.5)),
      maxRackAngularSpeedRadS: Math.max(0.5, Number(c.steeringRackMaxAngularSpeedRadS ?? 3.2)),
      maxRackAngularAccelRadS2: Math.max(10, Number(c.steeringRackMaxAngularAccelRadS2 ?? 180)),
      driverTorsionStiffnessNmPerRad: Math.max(0.1, Number(c.steeringColumnTorsionStiffnessNmPerRad ?? 4.0)),
      driverTorsionDampingNmsPerRad: Math.max(0, Number(c.steeringColumnTorsionDampingNmsPerRad ?? 0.18)),
      driverMaxTorqueNm: Math.max(1, Number(c.steeringDriverMaxTorqueNm ?? 8.0)),
      epsAssistParkingGain: Math.max(0, Number(c.steeringEpsParkingGain ?? 20.0)),
      epsAssistHighSpeedGain: Math.max(0, Number(c.steeringEpsHighSpeedGain ?? 9.0)),
      epsAssistFadeSpeedMs: Math.max(3, Number(c.steeringEpsFadeSpeedMs ?? 27.8)),
      epsMaxAssistTorqueNm: Math.max(1, Number(c.steeringEpsMaxAssistTorqueNm ?? 65)),
      stopStartFraction: PhysicsMath.clamp(Number(c.steeringStopStartFraction ?? 0.92), 0.70, 0.995),
      stopStiffnessNmPerRad: Math.max(100, Number(c.steeringStopStiffnessNmPerRad ?? 9000)),
      stopDampingNmsPerRad: Math.max(0, Number(c.steeringStopDampingNmsPerRad ?? 190)),
      tieRodStiffnessNmPerRad: Math.max(10000, Number(c.steeringTieRodStiffnessNmPerRad ?? 180000)),
      rackMountStiffnessNmPerRad: Math.max(10000, Number(c.steeringRackMountStiffnessNmPerRad ?? 250000)),
      controlArmBushingStiffnessNmPerRad: Math.max(10000, Number(c.steeringControlArmBushingStiffnessNmPerRad ?? 140000)),
      tireCarcassSteerStiffnessNmPerRad: Math.max(10000, Number(c.steeringTireCarcassStiffnessNmPerRad ?? 110000)),
      complianceDampingNmsPerRad: Math.max(50, Number(c.steeringComplianceDampingNmsPerRad ?? 1800)),
      maxComplianceRad: Math.max(0.001, Number(c.steeringMaxComplianceRad ?? 0.0065)),
    };
  }

  private makeTelemetry(): SteeringDynamicsTelemetry {
    return {
      driverInput: 0,
      targetSteeringWheelAngleRad: 0,
      steeringWheelAngleRad: 0,
      steeringWheelVelocityRadS: 0,
      localSteeringRatio: this.config.steeringRatioCenter,
      rackDisplacementM: 0,
      rackVelocityMS: 0,
      rackCenterAngleRad: 0,
      rackAngularVelocityRadS: 0,
      rackAngularAccelerationRadS2: 0,
      leftRoadWheelAngleRad: 0,
      rightRoadWheelAngleRad: 0,
      leftComplianceRad: 0,
      rightComplianceRad: 0,
      leftMechanicalTrailM: 0,
      rightMechanicalTrailM: 0,
      epsAssistGain: this.config.epsAssistParkingGain,
      torques: ZERO_TORQUES(),
    };
  }

  /** Local steering-wheel/road-wheel ratio. The BMW-published 14.2:1 is the center reference. */
  public steeringRatioAt(centerAngleRad: number): number {
    const n = PhysicsMath.clamp(
      Math.abs(centerAngleRad) / this.config.maxRoadWheelAngleRad,
      0,
      1
    );
    const shaped = n * n;
    return PhysicsMath.lerp(
      this.config.steeringRatioCenter,
      this.config.steeringRatioAtLock,
      shaped
    );
  }

  /** Integral of the variable steering ratio, so steering-wheel angle remains continuous. */
  public steeringWheelAngleForRack(centerAngleRad: number): number {
    const q = PhysicsMath.clamp(
      centerAngleRad,
      -this.config.maxRoadWheelAngleRad,
      this.config.maxRoadWheelAngleRad
    );
    const deltaRatio = this.config.steeringRatioAtLock - this.config.steeringRatioCenter;
    const qMaxSq = this.config.maxRoadWheelAngleRad * this.config.maxRoadWheelAngleRad;
    return this.config.steeringRatioCenter * q + (deltaRatio * q * q * q) / (3 * qMaxSq);
  }

  public maxSteeringWheelAngleRad(): number {
    return Math.abs(this.steeringWheelAngleForRack(this.config.maxRoadWheelAngleRad));
  }

  /** Pure Ackermann command before compliance and suspension bump-steer are added. */
  public ackermannForCenter(centerAngleRad: number): { left: number; right: number } {
    const delta = PhysicsMath.clamp(
      centerAngleRad,
      -this.config.maxRoadWheelAngleRad,
      this.config.maxRoadWheelAngleRad
    );
    if (Math.abs(delta) < 1e-8) return { left: 0, right: 0 };

    const L = this.config.wheelbaseM;
    const W = this.config.frontTrackM;
    const tanDelta = Math.tan(Math.abs(delta));
    let inner = Math.atan((L * tanDelta) / Math.max(0.10, L - 0.5 * W * tanDelta));
    let outer = Math.atan((L * tanDelta) / Math.max(0.10, L + 0.5 * W * tanDelta));
    inner = PhysicsMath.lerp(Math.abs(delta), inner, this.config.ackermannRatio);
    outer = PhysicsMath.lerp(Math.abs(delta), outer, this.config.ackermannRatio);

    return delta > 0
      ? { left: inner, right: outer }
      : { left: -outer, right: -inner };
  }

  private ackermannDerivative(centerAngleRad: number, sideIndex: 0 | 1): number {
    const h = 1e-4;
    const plus = this.ackermannForCenter(centerAngleRad + h);
    const minus = this.ackermannForCenter(centerAngleRad - h);
    const p = sideIndex === 0 ? plus.left : plus.right;
    const m = sideIndex === 0 ? minus.left : minus.right;
    return (p - m) / (2 * h);
  }

  private roadTorqueForFrontWheel(index: 0 | 1): {
    tireNm: number;
    mechanicalNm: number;
    trailM: number;
  } {
    const pose = this.getPoses()?.[index];
    const wheel = this.vehicle.wheels[index];
    const tire = wheel.lastTireOutput;
    if (!pose) {
      const trailM = this.vehicle.config.wheelRadius * Math.tan(7.2 * Math.PI / 180);
      return {
        tireNm: tire.aligningTorque,
        mechanicalNm: -tire.fy * trailM,
        trailM,
      };
    }

    const axis = PhysicsMath.vec3Normalize(pose.steeringAxisBody);
    const wheelUp = PhysicsMath.vec3Normalize(pose.upBody);
    const satProjection = PhysicsMath.vec3Dot(axis, wheelUp);
    const tireNm = tire.aligningTorque * satProjection;

    // Mechanical trail and scrub are not coefficients here. Project the actual
    // contact-patch force moment about the solved steering axis. The ground-plane
    // intersection follows directly from caster/KPI geometry already used by the
    // suspension solver.
    const contact = PhysicsMath.vec3(
      pose.hubCenterBody.x,
      pose.hubCenterBody.y - this.vehicle.config.wheelRadius,
      pose.hubCenterBody.z
    );
    const axisY = Math.abs(axis.y) > 1e-7 ? axis.y : Math.sign(axis.y || 1) * 1e-7;
    const tGround = (contact.y - pose.lowerBallJointBody.y) / axisY;
    const axisGround = PhysicsMath.vec3Add(
      pose.lowerBallJointBody,
      PhysicsMath.vec3Scale(axis, tGround)
    );
    const r = PhysicsMath.vec3Sub(contact, axisGround);
    const forceBody = PhysicsMath.vec3Add(
      PhysicsMath.vec3Scale(pose.forwardBody, tire.fx),
      PhysicsMath.vec3Scale(pose.lateralBody, tire.fy)
    );
    const mechanicalNm = PhysicsMath.vec3Dot(PhysicsMath.vec3Cross(r, forceBody), axis);
    const trailM = axisGround.z - pose.hubCenterBody.z;
    return { tireNm, mechanicalNm, trailM };
  }

  private updateCompliance(wheelIndex: 0 | 1, roadAxisTorqueNm: number, dt: number): number {
    const stiffnesses = [
      this.config.tieRodStiffnessNmPerRad,
      this.config.rackMountStiffnessNmPerRad,
      this.config.controlArmBushingStiffnessNmPerRad,
      this.config.tireCarcassSteerStiffnessNmPerRad,
    ];
    const states = this.complianceComponents[wheelIndex];
    for (let i = 0; i < states.length; i++) {
      // Kelvin-Voigt element: C*d(delta)/dt + K*delta = applied steering-axis torque.
      const rate = (roadAxisTorqueNm - stiffnesses[i] * states[i]) /
        this.config.complianceDampingNmsPerRad;
      states[i] += PhysicsMath.clamp(rate, -0.35, 0.35) * dt;
      const componentLimit = this.config.maxComplianceRad * 0.70;
      states[i] = PhysicsMath.clamp(states[i], -componentLimit, componentLimit);
    }
    return PhysicsMath.clamp(
      states.reduce((sum, value) => sum + value, 0),
      -this.config.maxComplianceRad,
      this.config.maxComplianceRad
    );
  }

  private epsGain(speedMs: number): number {
    const x = PhysicsMath.clamp(Math.abs(speedMs) / this.config.epsAssistFadeSpeedMs, 0, 1);
    const smooth = x * x * (3 - 2 * x);
    return PhysicsMath.lerp(
      this.config.epsAssistParkingGain,
      this.config.epsAssistHighSpeedGain,
      smooth
    );
  }

  private stopTorque(): number {
    const qAbs = Math.abs(this.rackCenterAngleRad);
    const start = this.config.maxRoadWheelAngleRad * this.config.stopStartFraction;
    if (qAbs <= start) return 0;
    const penetration = qAbs - start;
    const direction = Math.sign(this.rackCenterAngleRad) || 1;
    const outwardVelocity = Math.max(0, this.rackAngularVelocityRadS * direction);
    return -direction * (
      this.config.stopStiffnessNmPerRad * penetration +
      this.config.stopDampingNmsPerRad * outwardVelocity
    );
  }

  public update(
    steerInput: number,
    forwardSpeedMs: number,
    dt: number
  ): { steerFL: number; steerFR: number; centerAngle: number } {
    if (!(dt > 0)) {
      const base = this.ackermannForCenter(this.rackCenterAngleRad);
      return { steerFL: base.left, steerFR: base.right, centerAngle: this.rackCenterAngleRad };
    }

    const input = PhysicsMath.clamp(Number.isFinite(steerInput) ? steerInput : 0, -1, 1);
    const leftRoad = this.roadTorqueForFrontWheel(0);
    const rightRoad = this.roadTorqueForFrontWheel(1);
    const leftDerivative = this.ackermannDerivative(this.rackCenterAngleRad, 0);
    const rightDerivative = this.ackermannDerivative(this.rackCenterAngleRad, 1);

    const tireSelfAligningRoadNm =
      leftRoad.tireNm * leftDerivative + rightRoad.tireNm * rightDerivative;
    const casterMechanicalTrailRoadNm =
      leftRoad.mechanicalNm * leftDerivative + rightRoad.mechanicalNm * rightDerivative;
    const roadGeneralizedTorqueNm = tireSelfAligningRoadNm + casterMechanicalTrailRoadNm;

    const leftCompliance = this.updateCompliance(
      0,
      leftRoad.tireNm + leftRoad.mechanicalNm,
      dt
    );
    const rightCompliance = this.updateCompliance(
      1,
      rightRoad.tireNm + rightRoad.mechanicalNm,
      dt
    );

    const steeringWheelAngleRad = this.steeringWheelAngleForRack(this.rackCenterAngleRad);
    const ratio = this.steeringRatioAt(this.rackCenterAngleRad);
    const steeringWheelVelocityRadS = ratio * this.rackAngularVelocityRadS;
    const targetSteeringWheelAngleRad = input * this.maxSteeringWheelAngleRad();
    const driverTorqueNm = PhysicsMath.clamp(
      this.config.driverTorsionStiffnessNmPerRad *
        (targetSteeringWheelAngleRad - steeringWheelAngleRad) -
        this.config.driverTorsionDampingNmsPerRad * steeringWheelVelocityRadS,
      -this.config.driverMaxTorqueNm,
      this.config.driverMaxTorqueNm
    );

    const assistGain = this.epsGain(forwardSpeedMs);
    const epsAssistSteeringWheelNm = PhysicsMath.clamp(
      driverTorqueNm * assistGain,
      -this.config.epsMaxAssistTorqueNm,
      this.config.epsMaxAssistTorqueNm
    );
    const driverAndAssistRoadNm = (driverTorqueNm + epsAssistSteeringWheelNm) * ratio;

    const steeringDampingRoadNm = -this.config.rackDampingNmsPerRad * this.rackAngularVelocityRadS;
    const steeringFrictionRoadNm = -this.config.rackFrictionNm *
      Math.tanh(this.rackAngularVelocityRadS / 0.08);
    const steeringStopRoadNm = this.stopTorque();

    const netRackRoadNm =
      driverAndAssistRoadNm +
      roadGeneralizedTorqueNm +
      steeringDampingRoadNm +
      steeringFrictionRoadNm +
      steeringStopRoadNm;

    this.rackAngularAccelerationRadS2 = PhysicsMath.clamp(
      netRackRoadNm / this.config.rackEquivalentInertiaKgm2,
      -this.config.maxRackAngularAccelRadS2,
      this.config.maxRackAngularAccelRadS2
    );
    this.rackAngularVelocityRadS = PhysicsMath.clamp(
      this.rackAngularVelocityRadS + this.rackAngularAccelerationRadS2 * dt,
      -this.config.maxRackAngularSpeedRadS,
      this.config.maxRackAngularSpeedRadS
    );
    this.rackCenterAngleRad += this.rackAngularVelocityRadS * dt;

    if (this.rackCenterAngleRad > this.config.maxRoadWheelAngleRad) {
      this.rackCenterAngleRad = this.config.maxRoadWheelAngleRad;
      if (this.rackAngularVelocityRadS > 0) this.rackAngularVelocityRadS = 0;
    } else if (this.rackCenterAngleRad < -this.config.maxRoadWheelAngleRad) {
      this.rackCenterAngleRad = -this.config.maxRoadWheelAngleRad;
      if (this.rackAngularVelocityRadS < 0) this.rackAngularVelocityRadS = 0;
    }

    const base = this.ackermannForCenter(this.rackCenterAngleRad);
    const maxIndividualAngle = this.config.maxRoadWheelAngleRad + 0.12;
    const steerFL = PhysicsMath.clamp(
      base.left + leftCompliance,
      -maxIndividualAngle,
      maxIndividualAngle
    );
    const steerFR = PhysicsMath.clamp(
      base.right + rightCompliance,
      -maxIndividualAngle,
      maxIndividualAngle
    );

    const dxDq = this.config.rackHalfTravelM / this.config.maxRoadWheelAngleRad;
    const steeringInertiaRoadNm = -this.config.rackEquivalentInertiaKgm2 *
      this.rackAngularAccelerationRadS2;
    const ffbReadySteeringWheelNm =
      (tireSelfAligningRoadNm +
        casterMechanicalTrailRoadNm +
        steeringDampingRoadNm +
        steeringFrictionRoadNm +
        steeringStopRoadNm +
        steeringInertiaRoadNm) / Math.max(1, ratio) +
      epsAssistSteeringWheelNm;

    this.telemetry = {
      driverInput: input,
      targetSteeringWheelAngleRad,
      steeringWheelAngleRad: this.steeringWheelAngleForRack(this.rackCenterAngleRad),
      steeringWheelVelocityRadS: this.steeringRatioAt(this.rackCenterAngleRad) *
        this.rackAngularVelocityRadS,
      localSteeringRatio: this.steeringRatioAt(this.rackCenterAngleRad),
      rackDisplacementM: this.rackCenterAngleRad * dxDq,
      rackVelocityMS: this.rackAngularVelocityRadS * dxDq,
      rackCenterAngleRad: this.rackCenterAngleRad,
      rackAngularVelocityRadS: this.rackAngularVelocityRadS,
      rackAngularAccelerationRadS2: this.rackAngularAccelerationRadS2,
      leftRoadWheelAngleRad: steerFL,
      rightRoadWheelAngleRad: steerFR,
      leftComplianceRad: leftCompliance,
      rightComplianceRad: rightCompliance,
      leftMechanicalTrailM: leftRoad.trailM,
      rightMechanicalTrailM: rightRoad.trailM,
      epsAssistGain: assistGain,
      torques: {
        tireSelfAligningRoadNm,
        casterMechanicalTrailRoadNm,
        steeringDampingRoadNm,
        steeringFrictionRoadNm,
        steeringInertiaRoadNm,
        epsAssistSteeringWheelNm,
        driverSteeringWheelNm: driverTorqueNm,
        steeringStopRoadNm,
        netRackRoadNm,
        ffbReadySteeringWheelNm,
      },
    };

    return { steerFL, steerFR, centerAngle: this.rackCenterAngleRad };
  }
}
