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
    dt: number,
    reflectedDrivelineInertia: number = 0
  ): TireForceOutput {
    if (dt <= 0) return this.lastTireOutput;

    const fz = Math.max(0, verticalLoad);
    const brakeRequest = Math.max(0, hydraulicBrakeTorque) + Math.max(0, handbrakeTorque);
    const roadOmega = longitudinalVelocity / this.radius;

    // A free-rolling wheel is kinematically constrained very strongly by the road.
    // At 120 Hz the explicit tire-torque integration could otherwise overshoot the
    // rolling speed every frame and invent large alternating longitudinal forces.
    // Pre-coupling only wheels with essentially zero axle/brake torque preserves
    // driven-wheel slip while making unpowered rolling behavior physically stable.
    if (Math.abs(driveTorque) < 8 && brakeRequest < 8 && fz > 20) {
      const trackingRate = Math.abs(longitudinalVelocity) < 5 ? 120 : 45;
      const trackingAlpha = 1 - Math.exp(-trackingRate * dt);
      this.angularVelocity += (roadOmega - this.angularVelocity) * trackingAlpha;
    }

    const wheelSurfaceSpeed = this.angularVelocity * this.radius;
    // A 2 m/s regularization floor prevents near-zero velocity from turning a
    // millimetric wheel-speed error into a huge slip ratio. This does not suppress
    // launch traction because driven-wheel surface speed can still exceed the floor.
    const speedForSlip = Math.max(2.0, Math.abs(longitudinalVelocity), Math.abs(wheelSurfaceSpeed) * 0.35);

    this.rawSlipRatio = PhysicsMath.clamp(
      (wheelSurfaceSpeed - longitudinalVelocity) / speedForSlip,
      -3,
      3
    );

    // Positive lateral velocity means the contact patch is moving to the right;
    // tire force must develop to the left, hence the negative slip-angle sign.
    const angleSpeedFloor = 0.9;
    this.rawSlipAngle = -Math.atan2(
      lateralVelocity,
      Math.max(angleSpeedFloor, Math.abs(longitudinalVelocity))
    );

    if (fz < 20) {
      const airborneDecay = Math.exp(-dt / 0.025);
      this.relaxationSlipAngle *= airborneDecay;
      this.relaxationSlipRatio *= airborneDecay;
      this.transientFx *= airborneDecay;
      this.transientFy *= airborneDecay;
      this.transientMz *= airborneDecay;
    } else {
      const sigma = Math.max(0.035, this.tireConfig.relaxationLength);
      const relaxationTravel = Math.max(2.0, Math.abs(longitudinalVelocity)) * dt;
      const slipAlpha = 1 - Math.exp(-relaxationTravel / sigma);
      this.relaxationSlipAngle += (this.rawSlipAngle - this.relaxationSlipAngle) * slipAlpha;
      this.relaxationSlipRatio += (this.rawSlipRatio - this.relaxationSlipRatio) * slipAlpha;
    }

    const optimalTemp = this.tireConfig.optimalTemp;
    const tempError = Math.abs(this.temperature - optimalTemp);
    // Street/performance tires still have substantial grip at ambient temperature.
    // The old curve left a 25 C tire at ~78% grip, making a normal road car feel
    // artificially icy until warmed like a racing slick.
    const thermalGrip = PhysicsMath.clamp(1.02 - tempError * 0.0018, 0.88, 1.02);
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

    const sigmaForce = Math.max(0.025, this.tireConfig.relaxationLength * 0.55);
    const forceTravel = Math.max(2.5, Math.abs(longitudinalVelocity)) * dt;
    const forceAlpha = 1 - Math.exp(-forceTravel / sigmaForce);
    this.transientFx += (target.fx - this.transientFx) * forceAlpha;
    this.transientFy += (target.fy - this.transientFy) * forceAlpha;
    this.transientMz += (target.aligningTorque - this.transientMz) * forceAlpha;

    let rrForce = 0;
    if (Math.abs(longitudinalVelocity) > 0.15) {
      rrForce = -Math.sign(longitudinalVelocity) * Math.max(0, rollingResistance) * fz;
    }
    let fx = this.transientFx + rrForce;
    let fy = this.transientFy;

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
    const spinReference = Math.abs(this.angularVelocity) > 0.35 ? this.angularVelocity : roadOmega;
    const brakeSign = Math.sign(spinReference);
    const brakeTorque = brakeRequest * brakeSign;
    const tireReactionTorque = fx * this.radius;
    const effectiveRotationalInertia = this.inertia + Math.max(0, reflectedDrivelineInertia);
    const angularAccel = PhysicsMath.clamp(
      (driveTorque - brakeTorque - tireReactionTorque) / effectiveRotationalInertia,
      -4500,
      4500
    );
    const omegaBefore = this.angularVelocity;
    this.angularVelocity += angularAccel * dt;

    // When tire reaction alone would numerically shoot a lightly torqued wheel
    // through the exact rolling speed in one frame, land on rolling speed rather
    // than oscillating to the opposite side of the slip curve.
    const beforeError = omegaBefore - roadOmega;
    const afterError = this.angularVelocity - roadOmega;
    if (Math.abs(driveTorque) < 20 && brakeRequest < 20 && beforeError * afterError < 0) {
      this.angularVelocity = roadOmega;
    }

    if (brakeRequest > 0 && Math.abs(longitudinalVelocity) < 1.0 && Math.sign(this.angularVelocity) !== Math.sign(spinReference)) {
      this.angularVelocity = 0;
    }

    if (Math.abs(longitudinalVelocity) < 1.0 && Math.abs(driveTorque) < 15 && brakeRequest < 15) {
      const sync = 1 - Math.exp(-14 * dt);
      this.angularVelocity += (roadOmega - this.angularVelocity) * sync;
    }

    this.rotationAngle += this.angularVelocity * dt;
    if (Math.abs(this.rotationAngle) > Math.PI * 1000) this.rotationAngle %= Math.PI * 2;

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
