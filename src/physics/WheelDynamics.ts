import { PhysicsMath } from './math/PhysicsMath';
import { TireModel, TireModelConfig, TireForceOutput } from './TireModel';

export interface WheelDynamicsConfig {
  id: string;
  isFront: boolean;
  isLeft: boolean;
  radius: number;
  inertia: number;
  tireConfig: TireModelConfig;
}

const makeZeroTireOutput = (): TireForceOutput => ({
  fx: 0,
  fy: 0,
  aligningTorque: 0,
  pneumaticTrail: 0,
  sidewallDeflection: 0,
  tireSquishM: 0,
  isSkidding: false,
  skidIntensity: 0,
  frictionLimit: 0,
  gripUtilization: 0,
  effectiveMu: 0,
});

/**
 * One wheel rotational DOF + tire transient states.
 *
 * Raw geometric slip is deliberately NOT sent straight to the chassis. First
 * contact-patch slip relaxes over tire travel, then force itself relaxes over a
 * shorter carcass length. This turns an instantaneous steering command into a
 * short physical sequence: slip builds -> tire force builds -> chassis loads up.
 */
export class WheelDynamics {
  public readonly id: string;
  public readonly isFront: boolean;
  public readonly isLeft: boolean;
  public readonly radius: number;
  public readonly inertia: number;

  public steerAngle = 0;
  public rotationAngle = 0;
  public angularVelocity = 0;

  public rawSlipAngle = 0;
  public rawSlipRatio = 0;
  public relaxationSlipAngle = 0;
  public relaxationSlipRatio = 0;

  public temperature = 25;
  public pressurePsi: number;
  public wearPercent = 0;
  public brakeRotorTemp = 25;

  public lastTireOutput: TireForceOutput = makeZeroTireOutput();

  private tireModel: TireModel;
  private transientFx = 0;
  private transientFy = 0;
  private transientMz = 0;

  constructor(config: WheelDynamicsConfig) {
    this.id = config.id;
    this.isFront = config.isFront;
    this.isLeft = config.isLeft;
    this.radius = Math.max(0.05, config.radius);
    this.inertia = Math.max(0.05, config.inertia);
    this.tireModel = new TireModel(config.tireConfig);
    this.pressurePsi = config.tireConfig.basePressurePsi;
  }

  public get tireConfig(): TireModelConfig {
    return this.tireModel.config;
  }

  public set tireConfig(config: TireModelConfig) {
    this.tireModel.config = { ...config };
    if (!Number.isFinite(this.pressurePsi)) this.pressurePsi = config.basePressurePsi;
  }

  public reset(forwardSpeed: number = 0) {
    this.steerAngle = 0;
    this.rotationAngle = 0;
    this.angularVelocity = forwardSpeed / this.radius;
    this.rawSlipAngle = 0;
    this.rawSlipRatio = 0;
    this.relaxationSlipAngle = 0;
    this.relaxationSlipRatio = 0;
    this.transientFx = 0;
    this.transientFy = 0;
    this.transientMz = 0;
    this.temperature = 25;
    this.pressurePsi = this.tireConfig.basePressurePsi;
    this.wearPercent = 0;
    this.brakeRotorTemp = 25;
    this.lastTireOutput = makeZeroTireOutput();
    this.tireModel.reset();
  }

  public update(
    longitudinalVelocity: number,
    lateralVelocity: number,
    verticalLoad: number,
    camberDeg: number,
    driveTorque: number,
    hydraulicBrakeTorque: number,
    handbrakeTorque: number,
    surfaceFriction: number,
    rollingResistance: number,
    dt: number
  ): TireForceOutput {
    if (dt <= 0) return this.lastTireOutput;

    const fz = Math.max(0, verticalLoad);
    const wheelSurfaceSpeed = this.angularVelocity * this.radius;
    const speedForSlip = Math.max(1.2, Math.abs(longitudinalVelocity), Math.abs(wheelSurfaceSpeed) * 0.35);

    this.rawSlipRatio = PhysicsMath.clamp(
      (wheelSurfaceSpeed - longitudinalVelocity) / speedForSlip,
      -3,
      3
    );

    // Positive lateral velocity means the contact patch is moving to the right;
    // tire force must develop to the left, hence the negative slip-angle sign.
    const angleSpeedFloor = 0.7;
    this.rawSlipAngle = -Math.atan2(
      lateralVelocity,
      Math.max(angleSpeedFloor, Math.abs(longitudinalVelocity))
    );

    if (fz < 20) {
      // Airborne tire cannot retain a large contact patch force memory.
      const airborneDecay = Math.exp(-dt / 0.025);
      this.relaxationSlipAngle *= airborneDecay;
      this.relaxationSlipRatio *= airborneDecay;
      this.transientFx *= airborneDecay;
      this.transientFy *= airborneDecay;
      this.transientMz *= airborneDecay;
    } else {
      const sigma = Math.max(0.035, this.tireConfig.relaxationLength);
      // Relaxation is distance-based, but retain a small speed floor so parking
      // maneuvers still settle instead of freezing stale slip state indefinitely.
      const relaxationTravel = Math.max(2.0, Math.abs(longitudinalVelocity)) * dt;
      const slipAlpha = 1 - Math.exp(-relaxationTravel / sigma);
      this.relaxationSlipAngle += (this.rawSlipAngle - this.relaxationSlipAngle) * slipAlpha;
      this.relaxationSlipRatio += (this.rawSlipRatio - this.relaxationSlipRatio) * slipAlpha;
    }

    const optimalTemp = this.tireConfig.optimalTemp;
    const tempError = Math.abs(this.temperature - optimalTemp);
    const thermalGrip = PhysicsMath.clamp(1.03 - tempError * 0.0042, 0.72, 1.03);
    const wearGrip = PhysicsMath.clamp(1 - this.wearPercent * 0.0022, 0.70, 1.0);

    const target = this.tireModel.calculate({
      slipRatio: this.relaxationSlipRatio,
      slipAngle: this.relaxationSlipAngle,
      verticalLoad: fz,
      camberDeg,
      surfaceFriction,
      gripScale: thermalGrip * wearGrip,
      isLeft: this.isLeft,
    });

    // A second, shorter carcass-force relaxation prevents even a filtered slip
    // state from snapping force onto the chassis in one 120 Hz tick.
    const sigmaForce = Math.max(0.025, this.tireConfig.relaxationLength * 0.55);
    const forceTravel = Math.max(2.5, Math.abs(longitudinalVelocity)) * dt;
    const forceAlpha = 1 - Math.exp(-forceTravel / sigmaForce);
    this.transientFx += (target.fx - this.transientFx) * forceAlpha;
    this.transientFy += (target.fy - this.transientFy) * forceAlpha;
    this.transientMz += (target.aligningTorque - this.transientMz) * forceAlpha;

    // Rolling resistance is a road force, so it follows vehicle travel direction
    // and is kept small relative to the tire friction circle.
    let rrForce = 0;
    if (Math.abs(longitudinalVelocity) > 0.15) {
      rrForce = -Math.sign(longitudinalVelocity) * Math.max(0, rollingResistance) * fz;
    }
    let fx = this.transientFx + rrForce;
    let fy = this.transientFy;

    // The transient vector is still capped by the instantaneous load-sensitive
    // friction budget so a sudden unload cannot leave impossible "stored" force.
    const limit = Math.max(0, target.frictionLimit);
    const resultant = Math.hypot(fx, fy);
    if (limit > 0 && resultant > limit) {
      const scale = limit / resultant;
      fx *= scale;
      fy *= scale;
      this.transientFx *= scale;
      this.transientFy *= scale;
    }

    // Tire longitudinal force reacts back on the wheel rotational DOF.
    const brakeRequest = Math.max(0, hydraulicBrakeTorque) + Math.max(0, handbrakeTorque);
    const spinReference = Math.abs(this.angularVelocity) > 0.35 ? this.angularVelocity : longitudinalVelocity / this.radius;
    const brakeSign = Math.sign(spinReference);
    const brakeTorque = brakeRequest * brakeSign;
    const tireReactionTorque = fx * this.radius;
    const angularAccel = PhysicsMath.clamp(
      (driveTorque - brakeTorque - tireReactionTorque) / this.inertia,
      -6000,
      6000
    );
    this.angularVelocity += angularAccel * dt;

    // Do not let brakes numerically drive a nearly stopped wheel backwards.
    if (brakeRequest > 0 && Math.abs(longitudinalVelocity) < 1.0 && Math.sign(this.angularVelocity) !== Math.sign(spinReference)) {
      this.angularVelocity = 0;
    }

    // At walking speed with little torque, gently synchronize wheel speed to
    // road speed to avoid slip-ratio chatter around zero.
    if (Math.abs(longitudinalVelocity) < 1.0 && Math.abs(driveTorque) < 15 && brakeRequest < 15) {
      const sync = 1 - Math.exp(-10 * dt);
      this.angularVelocity += (longitudinalVelocity / this.radius - this.angularVelocity) * sync;
    }

    this.rotationAngle += this.angularVelocity * dt;
    if (Math.abs(this.rotationAngle) > Math.PI * 1000) this.rotationAngle %= Math.PI * 2;

    // Lightweight tire/brake thermal state. It matters because repeated sliding
    // should not preserve identical grip forever, but the rates are intentionally slow.
    const slipEnergy = Math.abs(fx * (wheelSurfaceSpeed - longitudinalVelocity)) + Math.abs(fy * lateralVelocity);
    const heatIn = slipEnergy * 0.00005;
    const cooling = (this.temperature - 25) * (0.020 + Math.abs(longitudinalVelocity) * 0.0025);
    this.temperature += (heatIn - cooling) * dt;
    this.temperature = PhysicsMath.clamp(this.temperature, 20, 180);
    this.pressurePsi = this.tireConfig.basePressurePsi + Math.max(0, this.temperature - 25) * 0.035;
    this.wearPercent = PhysicsMath.clamp(this.wearPercent + slipEnergy * 1.5e-9 * dt, 0, 100);

    const brakePower = brakeRequest * Math.abs(this.angularVelocity);
    this.brakeRotorTemp += (brakePower * 0.00022 - (this.brakeRotorTemp - 25) * 0.08) * dt;
    this.brakeRotorTemp = PhysicsMath.clamp(this.brakeRotorTemp, 20, 900);

    const transientResultant = Math.hypot(fx, fy);
    const gripUtilization = limit > 0 ? PhysicsMath.clamp(transientResultant / limit, 0, 1.5) : 0;
    this.lastTireOutput = {
      ...target,
      fx,
      fy,
      aligningTorque: this.transientMz,
      gripUtilization,
      isSkidding: target.isSkidding || gripUtilization > 0.99,
      skidIntensity: PhysicsMath.clamp(Math.max(target.skidIntensity, gripUtilization - 0.78), 0, 1),
    };

    return this.lastTireOutput;
  }
}
