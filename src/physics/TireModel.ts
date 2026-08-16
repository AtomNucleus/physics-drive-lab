import { PhysicsMath } from './math/PhysicsMath';

export interface TireModelConfig {
  baseGrip: number;
  stiffnessB: number;
  loadSensitivity: number;
  slideFrictionMultiplier: number;
  relaxationLength: number;
  longitudinalRelaxationLength?: number;
  longitudinalForceRelaxationLength?: number;
  pneumaticTrailMax: number;
  camberStiffness: number;
  optimalTemp: number;
  basePressurePsi: number;
  sidewallStiffness?: number;
  verticalStiffness?: number;
  referenceLoadN?: number;
}

export interface TireForceOutput {
  fx: number;
  fy: number;
  aligningTorque: number;
  pneumaticTrail: number;
  sidewallDeflection: number;
  tireSquishM: number;
  isSkidding: boolean;
  skidIntensity: number;
  frictionLimit: number;
  gripUtilization: number;
  effectiveMu: number;
}

export interface TireForceInput {
  slipRatio: number;
  slipAngle: number;
  verticalLoad: number;
  camberDeg: number;
  surfaceFriction: number;
  gripScale?: number;
  isLeft?: boolean;
}

const zeroOutput = (): TireForceOutput => ({
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
 * Load-sensitive combined-slip tire model.
 *
 * The key realism property is that mu falls as Fz rises. A tire carrying twice
 * its reference load therefore produces less than twice the force. During roll
 * and pitch this changes the total axle grip instead of merely moving identical
 * grip from one tire to the other.
 */
export class TireModel {
  public config: TireModelConfig;

  private inferredReferenceLoadN = 3600;

  constructor(config: TireModelConfig) {
    this.config = { ...config };
  }

  public reset() {
    this.inferredReferenceLoadN = this.config.referenceLoadN || 3600;
  }

  public calculate(input: TireForceInput): TireForceOutput {
    const fz = Math.max(0, input.verticalLoad);
    if (fz < 5) return zeroOutput();

    // Slowly learn a representative corner load if the caller did not provide one.
    // This keeps load sensitivity sensible across the 1,220-2,050 kg presets.
    if (!this.config.referenceLoadN && fz > 500 && fz < 10000) {
      this.inferredReferenceLoadN += (fz - this.inferredReferenceLoadN) * 0.0025;
    }
    const referenceLoad = Math.max(800, this.config.referenceLoadN || this.inferredReferenceLoadN);
    const loadRatio = Math.max(0.08, fz / referenceLoad);

    // Convert the legacy "mu reduction per N" tuning knob into a load exponent.
    // At the default 3.5e-5 and ~3.6 kN reference load, exponent ~= 0.126.
    const loadExponent = PhysicsMath.clamp(this.config.loadSensitivity * referenceLoad, 0, 0.32);
    const loadSensitivityScale = Math.pow(loadRatio, -loadExponent);

    const gripScale = input.gripScale ?? 1;
    const effectiveMu = Math.max(
      0.05,
      this.config.baseGrip * Math.max(0.05, input.surfaceFriction) * gripScale * loadSensitivityScale
    );
    const peakForce = effectiveMu * fz;

    const kappa = PhysicsMath.clamp(input.slipRatio, -3, 3);
    const alpha = PhysicsMath.clamp(input.slipAngle, -1.2, 1.2);

    // Smooth Pacejka-like saturation. StiffnessB remains the user's primary
    // response tuning control, but lateral B is softened so turn-in is progressive.
    const bLong = Math.max(1.0, this.config.stiffnessB * 0.72);
    const bLat = Math.max(1.0, this.config.stiffnessB * 0.52);
    const pureFx = peakForce * Math.sin(1.55 * Math.atan(bLong * kappa));
    let pureFy = peakForce * Math.sin(1.35 * Math.atan(bLat * alpha));

    // Small, correctly mirrored camber thrust contribution.
    const signedCamber = (input.isLeft ? -1 : 1) * input.camberDeg;
    const camberThrust = PhysicsMath.clamp(
      signedCamber * this.config.camberStiffness,
      -peakForce * 0.12,
      peakForce * 0.12
    );
    pureFy += camberThrust;

    // Friction ellipse: acceleration/braking consumes lateral capacity and vice versa.
    const normalized = Math.sqrt((pureFx / Math.max(1, peakForce)) ** 2 + (pureFy / Math.max(1, peakForce)) ** 2);
    let fx = pureFx;
    let fy = pureFy;
    if (normalized > 1) {
      const inv = 1 / normalized;
      fx *= inv;
      fy *= inv;
    }

    // Past the useful slip window, real tires settle onto a lower sliding coefficient.
    const slipSeverity = Math.max(Math.abs(kappa) / 0.18, Math.abs(alpha) / 0.16);
    const slideBlend = PhysicsMath.clamp((slipSeverity - 1.0) / 2.2, 0, 1);
    const slideScale = PhysicsMath.lerp(1.0, PhysicsMath.clamp(this.config.slideFrictionMultiplier, 0.45, 1.0), slideBlend);
    fx *= slideScale;
    fy *= slideScale;

    const resultant = Math.hypot(fx, fy);
    const frictionLimit = peakForce * slideScale;
    const gripUtilization = PhysicsMath.clamp(resultant / Math.max(1, frictionLimit), 0, 1.5);

    // Pneumatic trail falls away with slip angle and at the edge of the friction circle.
    const trailSlipFalloff = 1 / (1 + Math.pow(Math.abs(alpha) / 0.13, 1.7));
    const trailLoadScale = PhysicsMath.clamp(Math.pow(loadRatio, 0.08), 0.85, 1.18);
    const pneumaticTrail = this.config.pneumaticTrailMax * trailSlipFalloff * trailLoadScale;
    const aligningTorque = -fy * pneumaticTrail;

    const sidewallStiffness = Math.max(40000, this.config.sidewallStiffness || 180000);
    const verticalStiffness = Math.max(90000, this.config.verticalStiffness || 240000);
    const sidewallDeflection = PhysicsMath.clamp(fy / sidewallStiffness, -0.045, 0.045);
    const tireSquishM = PhysicsMath.clamp(fz / verticalStiffness, 0, 0.055);

    return {
      fx,
      fy,
      aligningTorque,
      pneumaticTrail,
      sidewallDeflection,
      tireSquishM,
      isSkidding: slipSeverity > 1.15 || gripUtilization > 0.98,
      skidIntensity: PhysicsMath.clamp(Math.max((slipSeverity - 0.7) / 2.3, gripUtilization - 0.78), 0, 1),
      frictionLimit,
      gripUtilization,
      effectiveMu,
    };
  }
}
