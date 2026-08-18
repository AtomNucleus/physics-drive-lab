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
  effectiveRackDampingNmsPerRad: number;
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
  overallSteeringRatio: number;
  rackEquivalentInertiaKgm2: number;
  rackDampingNmsPerRad: number;
  rackNearTargetDampingNmsPerRad: number;
  rackNearTargetDampingWindowRad: number;
  rackFrictionNm: number;
  maxRackAngularSpeedRadS: number;
  maxRackAngularAccelRadS2: number;
  integrationSubsteps: number;
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
  complianceStiffnessNmPerRad: number;
  complianceDampingNmsPerRad: number;
  maxComplianceRad: number;
}

interface RackTorqueState {
  tireSelfAligningRoadNm: number;
  casterMechanicalTrailRoadNm: number;
  driverSteeringWheelNm: number;
  epsAssistSteeringWheelNm: number;
  steeringDampingRoadNm: number;
  steeringFrictionRoadNm: number;
  steeringStopRoadNm: number;
  effectiveRackDampingNmsPerRad: number;
  netRackRoadNm: number;
}

const zeroTorques = (): SteeringTorqueBreakdown => ({
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
 * One-DOF physical steering rack driven through a compliant steering column.
 *
 * Driver input is a steering-wheel target. Column torsion and EPS generate torque;
 * torque accelerates the rack; Ackermann then produces independent FL/FR commands.
 * Tire self-aligning torque and the solved steering-axis force moment feed back into
 * the rack from the previous 120 Hz tire solve. There is deliberately no yaw-rate,
 * sideslip, or stability-state term anywhere in this subsystem.
 *
 * The rack/column loop is internally substepped. A stiff assisted steering system can
 * have a natural frequency high enough that one explicit 8.33 ms step creates a
 * numerical phase reversal (centering torque evaluated from the wrong side of zero).
 * Substepping only this one-DOF mechanism preserves the authoritative 120 Hz vehicle
 * timestep while keeping the physical torque loop numerically resolved.
 *
 * BMW publishes the G90's overall 14.2:1 steering ratio, but not its rack inertia,
 * damping, compliance stiffness, EPS map, rack travel, or variable-ratio curve.
 * Those unavailable quantities are internal solver calibrations only and must never
 * be presented as BMW validation targets.
 */
export class PhysicalSteeringSystem {
  private config: SteeringCalibration;
  private rackCenterAngleRad = 0;
  private rackAngularVelocityRadS = 0;
  private rackAngularAccelerationRadS2 = 0;
  private complianceRad: [number, number] = [0, 0];

  public telemetry: SteeringDynamicsTelemetry;

  constructor(
    private readonly vehicle: Vehicle,
    private readonly getPoses: () => [WheelKinematicPose, WheelKinematicPose, WheelKinematicPose, WheelKinematicPose]
  ) {
    this.config = this.readCalibration();
    this.telemetry = this.makeTelemetry();
  }

  public get steeringWheelAngle(): number {
    return this.telemetry.steeringWheelAngleRad;
  }

  public get rackPositionM(): number {
    return this.telemetry.rackDisplacementM;
  }

  public get state() {
    return {
      steeringWheelAngle: this.telemetry.steeringWheelAngleRad,
      rackPositionM: this.telemetry.rackDisplacementM,
    };
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
    this.complianceRad = [0, 0];
    this.telemetry = this.makeTelemetry();
  }

  private readCalibration(): SteeringCalibration {
    const c = this.vehicle.config as any;
    const maxRackRate = Number(c.steeringRackMaxAngularSpeedRadS ?? c.steerSpeed ?? 3.2);
    const maxRoadWheelAngleRad = Math.max(0.15, Number(c.maxSteerAngle ?? 0.58));
    return {
      wheelbaseM: Math.max(0.5, Number(c.wheelbase ?? 3.0)),
      frontTrackM: Math.max(0.5, Number(c.steeringFrontTrackM ?? c.trackWidthFront ?? c.trackWidth ?? 1.65)),
      ackermannRatio: PhysicsMath.clamp(Number(c.ackermannRatio ?? 0.9), 0, 1.15),
      maxRoadWheelAngleRad,
      rackHalfTravelM: Math.max(0.02, Number(c.steeringRackHalfTravelM ?? 0.070)),
      overallSteeringRatio: Math.max(5, Number(c.steeringRatioOverall ?? c.steeringRatioCenter ?? 15.0)),
      rackEquivalentInertiaKgm2: Math.max(0.2, Number(c.steeringRackEquivalentInertiaKgm2 ?? 5.8)),
      rackDampingNmsPerRad: Math.max(0, Number(c.steeringRackDampingNmsPerRad ?? 46)),
      rackNearTargetDampingNmsPerRad: Math.max(0, Number(c.steeringRackNearTargetDampingNmsPerRad ?? 420)),
      rackNearTargetDampingWindowRad: Math.max(
        0.01,
        Number(c.steeringRackNearTargetDampingWindowRad ?? maxRoadWheelAngleRad * 0.16)
      ),
      rackFrictionNm: Math.max(0, Number(c.steeringRackFrictionNm ?? 4.5)),
      maxRackAngularSpeedRadS: Math.max(0.5, maxRackRate),
      maxRackAngularAccelRadS2: Math.max(10, Number(c.steeringRackMaxAngularAccelRadS2 ?? 90)),
      integrationSubsteps: Math.round(PhysicsMath.clamp(Number(c.steeringIntegrationSubsteps ?? 6), 1, 12)),
      driverTorsionStiffnessNmPerRad: Math.max(0.1, Number(c.steeringColumnTorsionStiffnessNmPerRad ?? 14.0)),
      driverTorsionDampingNmsPerRad: Math.max(0, Number(c.steeringColumnTorsionDampingNmsPerRad ?? 0.18)),
      driverMaxTorqueNm: Math.max(1, Number(c.steeringDriverMaxTorqueNm ?? 8.0)),
      epsAssistParkingGain: Math.max(0, Number(c.steeringEpsParkingGain ?? 20.0)),
      epsAssistHighSpeedGain: Math.max(0, Number(c.steeringEpsHighSpeedGain ?? 9.0)),
      epsAssistFadeSpeedMs: Math.max(3, Number(c.steeringEpsFadeSpeedMs ?? 27.8)),
      epsMaxAssistTorqueNm: Math.max(1, Number(c.steeringEpsMaxAssistTorqueNm ?? 65)),
      stopStartFraction: PhysicsMath.clamp(Number(c.steeringStopStartFraction ?? 0.92), 0.70, 0.995),
      stopStiffnessNmPerRad: Math.max(100, Number(c.steeringStopStiffnessNmPerRad ?? 9000)),
      stopDampingNmsPerRad: Math.max(0, Number(c.steeringStopDampingNmsPerRad ?? 190)),
      complianceStiffnessNmPerRad: Math.max(10000, Number(c.steeringComplianceStiffnessNmPerRad ?? 65000)),
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
      localSteeringRatio: this.config.overallSteeringRatio,
      rackDisplacementM: 0,
      rackVelocityMS: 0,
      rackCenterAngleRad: 0,
      rackAngularVelocityRadS: 0,
      rackAngularAccelerationRadS2: 0,
      effectiveRackDampingNmsPerRad: this.config.rackDampingNmsPerRad,
      leftRoadWheelAngleRad: 0,
      rightRoadWheelAngleRad: 0,
      leftComplianceRad: 0,
      rightComplianceRad: 0,
      leftMechanicalTrailM: 0,
      rightMechanicalTrailM: 0,
      epsAssistGain: this.config.epsAssistParkingGain,
      torques: zeroTorques(),
    };
  }

  public steeringRatioAt(_centerAngleRad: number): number {
    return this.config.overallSteeringRatio;
  }

  public steeringWheelAngleForRack(centerAngleRad: number): number {
    return PhysicsMath.clamp(
      centerAngleRad,
      -this.config.maxRoadWheelAngleRad,
      this.config.maxRoadWheelAngleRad
    ) * this.config.overallSteeringRatio;
  }

  public maxSteeringWheelAngleRad(): number {
    return this.config.maxRoadWheelAngleRad * this.config.overallSteeringRatio;
  }

  /** Pure rack/Ackermann command before compliance and suspension bump steer. */
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

  private ackermannDerivative(centerAngleRad: number, side: 0 | 1): number {
    const h = 1e-4;
    const plus = this.ackermannForCenter(centerAngleRad + h);
    const minus = this.ackermannForCenter(centerAngleRad - h);
    return side === 0
      ? (plus.left - minus.left) / (2 * h)
      : (plus.right - minus.right) / (2 * h);
  }

  private roadTorqueForFrontWheel(index: 0 | 1) {
    const pose = this.getPoses()?.[index];
    const tire = this.vehicle.wheels[index].lastTireOutput;
    if (!pose) return { tireNm: tire.aligningTorque, mechanicalNm: 0, trailM: 0 };

    const axis = PhysicsMath.vec3Normalize(pose.steeringAxisBody);
    const wheelUp = PhysicsMath.vec3Normalize(pose.upBody);
    const tireNm = tire.aligningTorque * PhysicsMath.vec3Dot(axis, wheelUp);

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

  private updateCompliance(index: 0 | 1, roadAxisTorqueNm: number, dt: number): number {
    const rate = (
      roadAxisTorqueNm - this.config.complianceStiffnessNmPerRad * this.complianceRad[index]
    ) / this.config.complianceDampingNmsPerRad;
    this.complianceRad[index] += PhysicsMath.clamp(rate, -0.35, 0.35) * dt;
    this.complianceRad[index] = PhysicsMath.clamp(
      this.complianceRad[index],
      -this.config.maxComplianceRad,
      this.config.maxComplianceRad
    );
    return this.complianceRad[index];
  }

  private epsGain(speedMs: number): number {
    const x = PhysicsMath.clamp(Math.abs(speedMs) / this.config.epsAssistFadeSpeedMs, 0, 1);
    const smooth = x * x * (3 - 2 * x);
    return PhysicsMath.lerp(this.config.epsAssistParkingGain, this.config.epsAssistHighSpeedGain, smooth);
  }

  private stopTorque(): number {
    const qAbs = Math.abs(this.rackCenterAngleRad);
    const start = this.config.maxRoadWheelAngleRad * this.config.stopStartFraction;
    if (qAbs <= start) return 0;
    const direction = Math.sign(this.rackCenterAngleRad) || 1;
    const penetration = qAbs - start;
    const outwardRate = Math.max(0, this.rackAngularVelocityRadS * direction);
    return -direction * (
      this.config.stopStiffnessNmPerRad * penetration +
      this.config.stopDampingNmsPerRad * outwardRate
    );
  }

  private effectiveRackDamping(targetRackAngleRad: number): number {
    const error = Math.abs(targetRackAngleRad - this.rackCenterAngleRad);
    const normalized = PhysicsMath.clamp(
      1 - error / this.config.rackNearTargetDampingWindowRad,
      0,
      1
    );
    const smooth = normalized * normalized * (3 - 2 * normalized);
    return this.config.rackDampingNmsPerRad + this.config.rackNearTargetDampingNmsPerRad * smooth;
  }

  private computeRackTorques(
    targetSteeringWheelAngleRad: number,
    forwardSpeedMs: number,
    leftRoad: { tireNm: number; mechanicalNm: number },
    rightRoad: { tireNm: number; mechanicalNm: number }
  ): RackTorqueState {
    const ratio = this.config.overallSteeringRatio;
    const targetRackAngleRad = targetSteeringWheelAngleRad / ratio;
    const steeringWheelAngleRad = this.rackCenterAngleRad * ratio;
    const steeringWheelVelocityRadS = this.rackAngularVelocityRadS * ratio;
    const leftDerivative = this.ackermannDerivative(this.rackCenterAngleRad, 0);
    const rightDerivative = this.ackermannDerivative(this.rackCenterAngleRad, 1);
    const tireSelfAligningRoadNm = leftRoad.tireNm * leftDerivative + rightRoad.tireNm * rightDerivative;
    const casterMechanicalTrailRoadNm = leftRoad.mechanicalNm * leftDerivative + rightRoad.mechanicalNm * rightDerivative;
    const driverSteeringWheelNm = PhysicsMath.clamp(
      this.config.driverTorsionStiffnessNmPerRad * (targetSteeringWheelAngleRad - steeringWheelAngleRad) -
        this.config.driverTorsionDampingNmsPerRad * steeringWheelVelocityRadS,
      -this.config.driverMaxTorqueNm,
      this.config.driverMaxTorqueNm
    );
    const assistGain = this.epsGain(forwardSpeedMs);
    const epsAssistSteeringWheelNm = PhysicsMath.clamp(
      driverSteeringWheelNm * assistGain,
      -this.config.epsMaxAssistTorqueNm,
      this.config.epsMaxAssistTorqueNm
    );
    const inputRoadNm = (driverSteeringWheelNm + epsAssistSteeringWheelNm) * ratio;
    const effectiveRackDampingNmsPerRad = this.effectiveRackDamping(targetRackAngleRad);
    const steeringDampingRoadNm = -effectiveRackDampingNmsPerRad * this.rackAngularVelocityRadS;
    const steeringFrictionRoadNm = -this.config.rackFrictionNm * Math.tanh(this.rackAngularVelocityRadS / 0.08);
    const steeringStopRoadNm = this.stopTorque();
    const netRackRoadNm = inputRoadNm + tireSelfAligningRoadNm + casterMechanicalTrailRoadNm +
      steeringDampingRoadNm + steeringFrictionRoadNm + steeringStopRoadNm;

    return {
      tireSelfAligningRoadNm,
      casterMechanicalTrailRoadNm,
      driverSteeringWheelNm,
      epsAssistSteeringWheelNm,
      steeringDampingRoadNm,
      steeringFrictionRoadNm,
      steeringStopRoadNm,
      effectiveRackDampingNmsPerRad,
      netRackRoadNm,
    };
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
    const leftCompliance = this.updateCompliance(0, leftRoad.tireNm + leftRoad.mechanicalNm, dt);
    const rightCompliance = this.updateCompliance(1, rightRoad.tireNm + rightRoad.mechanicalNm, dt);
    const ratio = this.config.overallSteeringRatio;
    const targetSteeringWheelAngleRad = input * this.maxSteeringWheelAngleRad();

    // Hold the previous 120 Hz tire/road torque sample constant while resolving the
    // much stiffer one-DOF rack/column loop internally. This is a partitioned
    // substep, not a higher-rate vehicle simulation and not an artificial yaw aid.
    const substeps = this.config.integrationSubsteps;
    const subDt = dt / substeps;
    let rackTorques = this.computeRackTorques(targetSteeringWheelAngleRad, forwardSpeedMs, leftRoad, rightRoad);
    for (let substep = 0; substep < substeps; substep++) {
      rackTorques = this.computeRackTorques(targetSteeringWheelAngleRad, forwardSpeedMs, leftRoad, rightRoad);
      this.rackAngularAccelerationRadS2 = PhysicsMath.clamp(
        rackTorques.netRackRoadNm / this.config.rackEquivalentInertiaKgm2,
        -this.config.maxRackAngularAccelRadS2,
        this.config.maxRackAngularAccelRadS2
      );
      this.rackAngularVelocityRadS = PhysicsMath.clamp(
        this.rackAngularVelocityRadS + this.rackAngularAccelerationRadS2 * subDt,
        -this.config.maxRackAngularSpeedRadS,
        this.config.maxRackAngularSpeedRadS
      );
      this.rackCenterAngleRad += this.rackAngularVelocityRadS * subDt;

      if (this.rackCenterAngleRad > this.config.maxRoadWheelAngleRad) {
        this.rackCenterAngleRad = this.config.maxRoadWheelAngleRad;
        if (this.rackAngularVelocityRadS > 0) this.rackAngularVelocityRadS = 0;
      } else if (this.rackCenterAngleRad < -this.config.maxRoadWheelAngleRad) {
        this.rackCenterAngleRad = -this.config.maxRoadWheelAngleRad;
        if (this.rackAngularVelocityRadS < 0) this.rackAngularVelocityRadS = 0;
      }
    }

    // Re-evaluate telemetry from the final substep state so reported driver/EPS and
    // road torques are phase-consistent with the reported rack angle and velocity.
    rackTorques = this.computeRackTorques(targetSteeringWheelAngleRad, forwardSpeedMs, leftRoad, rightRoad);
    this.rackAngularAccelerationRadS2 = PhysicsMath.clamp(
      rackTorques.netRackRoadNm / this.config.rackEquivalentInertiaKgm2,
      -this.config.maxRackAngularAccelRadS2,
      this.config.maxRackAngularAccelRadS2
    );

    const base = this.ackermannForCenter(this.rackCenterAngleRad);
    const individualLimit = this.config.maxRoadWheelAngleRad + 0.12;
    const steerFL = PhysicsMath.clamp(base.left + leftCompliance, -individualLimit, individualLimit);
    const steerFR = PhysicsMath.clamp(base.right + rightCompliance, -individualLimit, individualLimit);

    const travelPerRad = this.config.rackHalfTravelM / this.config.maxRoadWheelAngleRad;
    const steeringInertiaRoadNm = -this.config.rackEquivalentInertiaKgm2 * this.rackAngularAccelerationRadS2;
    const ffbReadySteeringWheelNm = (
      rackTorques.tireSelfAligningRoadNm + rackTorques.casterMechanicalTrailRoadNm +
      rackTorques.steeringDampingRoadNm + rackTorques.steeringFrictionRoadNm +
      rackTorques.steeringStopRoadNm + steeringInertiaRoadNm
    ) / ratio + rackTorques.epsAssistSteeringWheelNm;

    this.telemetry = {
      driverInput: input,
      targetSteeringWheelAngleRad,
      steeringWheelAngleRad: this.rackCenterAngleRad * ratio,
      steeringWheelVelocityRadS: this.rackAngularVelocityRadS * ratio,
      localSteeringRatio: ratio,
      rackDisplacementM: this.rackCenterAngleRad * travelPerRad,
      rackVelocityMS: this.rackAngularVelocityRadS * travelPerRad,
      rackCenterAngleRad: this.rackCenterAngleRad,
      rackAngularVelocityRadS: this.rackAngularVelocityRadS,
      rackAngularAccelerationRadS2: this.rackAngularAccelerationRadS2,
      effectiveRackDampingNmsPerRad: rackTorques.effectiveRackDampingNmsPerRad,
      leftRoadWheelAngleRad: steerFL,
      rightRoadWheelAngleRad: steerFR,
      leftComplianceRad: leftCompliance,
      rightComplianceRad: rightCompliance,
      leftMechanicalTrailM: leftRoad.trailM,
      rightMechanicalTrailM: rightRoad.trailM,
      epsAssistGain: this.epsGain(forwardSpeedMs),
      torques: {
        tireSelfAligningRoadNm: rackTorques.tireSelfAligningRoadNm,
        casterMechanicalTrailRoadNm: rackTorques.casterMechanicalTrailRoadNm,
        steeringDampingRoadNm: rackTorques.steeringDampingRoadNm,
        steeringFrictionRoadNm: rackTorques.steeringFrictionRoadNm,
        steeringInertiaRoadNm,
        epsAssistSteeringWheelNm: rackTorques.epsAssistSteeringWheelNm,
        driverSteeringWheelNm: rackTorques.driverSteeringWheelNm,
        steeringStopRoadNm: rackTorques.steeringStopRoadNm,
        netRackRoadNm: rackTorques.netRackRoadNm,
        ffbReadySteeringWheelNm,
      },
    };

    return { steerFL, steerFR, centerAngle: this.rackCenterAngleRad };
  }
}
