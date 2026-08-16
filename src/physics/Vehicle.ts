import { VehicleConfig, VehicleState, WheelState, ControlInputs } from '../types';
import { Vec3, PhysicsMath } from './math/PhysicsMath';
import { RigidBody } from './RigidBody';
import { SuspensionSystem, SuspensionCornerConfig } from './Suspension';
import { WheelDynamics } from './WheelDynamics';
import { Powertrain } from './Powertrain';
import { DifferentialSystem } from './Differential';
import { BrakeSystem } from './Brakes';
import { DriverAidsSystem } from './DriverAids';
import { AerodynamicsSystem } from './Aero';
import { TelemetrySystem } from './Telemetry';
import { ISurfaceProvider, ProvingGroundSurfaceProvider } from './SurfaceProvider';

export class Vehicle {
  public config: VehicleConfig;
  public rigidBody: RigidBody;
  public suspension: SuspensionSystem;
  public wheels: [WheelDynamics, WheelDynamics, WheelDynamics, WheelDynamics];
  public powertrain: Powertrain;
  public differential: DifferentialSystem;
  public brakes: BrakeSystem;
  public driverAids: DriverAidsSystem;
  public aero: AerodynamicsSystem;
  public telemetry: TelemetrySystem;
  public surfaceProvider: ISurfaceProvider;

  // Visual / Debug Options
  public showForceVectors3D: boolean = true;
  private totalSimTime: number = 0;

  // Smoothing filters for G-forces
  private smoothedAx: number = 0;
  private smoothedAy: number = 0;
  private smoothedAz: number = 0;
  private exhaustFlameTimer: number = 0;

  constructor(config: VehicleConfig, surfaceProvider?: ISurfaceProvider) {
    this.config = { ...config };
    this.surfaceProvider = surfaceProvider || new ProvingGroundSurfaceProvider();

    // 1. Calculate Moments of Inertia from mass, wheelbase, track width, and CG height.
    // Body axes in this simulation are X=lateral (pitch axis), Y=vertical (yaw axis),
    // Z=longitudinal (roll axis). Keep the principal inertias aligned to those axes.
    const m = this.config.mass;
    const L = this.config.wheelbase;
    const W = this.config.trackWidth;
    const H = this.config.centerOfGravityHeight;

    // Approximate sprung chassis as a cuboid with empirical multipliers for the fact
    // that a real automobile is not a homogeneous box. Pitch inertia belongs on X;
    // roll inertia belongs on Z. This was previously reversed.
    const Ixx = (m / 12) * (L * L + H * H) * 1.5; // Pitch inertia
    const Iyy = (m / 12) * (L * L + W * W) * 1.1; // Yaw inertia
    const Izz = (m / 12) * (W * W + H * H) * 1.6; // Roll inertia

    this.rigidBody = new RigidBody(
      {
        mass: m,
        inertia: PhysicsMath.vec3(Ixx, Iyy, Izz),
        centerOfGravityHeight: H,
      },
      PhysicsMath.vec3(0, H + 0.35, 0),
      0
    );

    this.suspension = new SuspensionSystem();

    // 2. Instantiate 4 Wheels [FL, FR, RL, RR]
    const tireRadius = this.config.wheelRadius;
    const wheelInertia = this.config.wheelInertia;

    const tireConfigFront = {
      baseGrip: this.config.tireGripFront,
      stiffnessB: this.config.tireStiffness,
      loadSensitivity: this.config.tireLoadSensitivity,
      slideFrictionMultiplier: this.config.slideFrictionMultiplier,
      relaxationLength: this.config.relaxationLength,
      pneumaticTrailMax: this.config.tirePneumaticTrailMax,
      camberStiffness: 85,
      optimalTemp: this.config.optimalTireTemp,
      basePressurePsi: this.config.tireBasePressure,
    };

    const tireConfigRear = {
      baseGrip: this.config.tireGripRear,
      stiffnessB: this.config.tireStiffness,
      loadSensitivity: this.config.tireLoadSensitivity,
      slideFrictionMultiplier: this.config.slideFrictionMultiplier,
      relaxationLength: this.config.relaxationLength,
      pneumaticTrailMax: this.config.tirePneumaticTrailMax,
      camberStiffness: 85,
      optimalTemp: this.config.optimalTireTemp,
      basePressurePsi: this.config.tireBasePressure,
    };

    this.wheels = [
      new WheelDynamics({ id: 'FL', isFront: true, isLeft: true, radius: tireRadius, inertia: wheelInertia, tireConfig: tireConfigFront }),
      new WheelDynamics({ id: 'FR', isFront: true, isLeft: false, radius: tireRadius, inertia: wheelInertia, tireConfig: tireConfigFront }),
      new WheelDynamics({ id: 'RL', isFront: false, isLeft: true, radius: tireRadius, inertia: wheelInertia, tireConfig: tireConfigRear }),
      new WheelDynamics({ id: 'RR', isFront: false, isLeft: false, radius: tireRadius, inertia: wheelInertia, tireConfig: tireConfigRear }),
    ];

    // 3. Powertrain
    this.powertrain = new Powertrain({
      maxTorque: this.config.maxTorque,
      idleRpm: this.config.idleRpm,
      maxRpm: this.config.maxRpm,
      revLimiterRpm: this.config.revLimiterRpm,
      flywheelInertia: this.config.flywheelInertia,
      engineBrakingTorque: this.config.engineBrakingTorque,
      clutchBiteRate: this.config.clutchBiteRate,
      turboBoostMaxPsi: this.config.turboBoostMaxPsi,
      turboSpoolRate: this.config.turboSpoolRate,
      wastegatePressurePsi: this.config.wastegatePressurePsi,
      reverseRatio: this.config.reverseRatio,
      forwardGearRatios: this.config.forwardGearRatios,
      gearRatios: this.config.gearRatios,
      finalDriveRatio: this.config.finalDriveRatio,
      maxClutchTorque: this.config.maxClutchTorque,
      transmissionEfficiency: this.config.transmissionEfficiency,
      autoBlipDownshift: this.config.autoBlipDownshift,
    });

    // 4. Differential
    this.differential = new DifferentialSystem({
      type: this.config.differentialType,
      powerRamp: this.config.diffPowerRamp,
      coastRamp: this.config.diffCoastRamp,
      preloadTorque: this.config.diffPreloadTorque,
      drivetrain: this.config.drivetrain,
      frontTorqueRatio: (this.config as any).centerFrontTorqueRatio,
    });

    // 5. Brakes
    this.brakes = new BrakeSystem({
      maxBrakeTorque: this.config.brakeForce,
      handbrakeTorque: this.config.handbrakeForce,
      frontBias: this.config.brakeBiasFront,
    });

    // 6. Driver Aids
    this.driverAids = new DriverAidsSystem({
      absMode: this.config.absMode,
      tcsMode: this.config.tcsMode,
      wheelbase: this.config.wheelbase,
      trackWidth: this.config.trackWidth,
      ackermannRatio: this.config.ackermannRatio,
      maxSteerAngle: this.config.maxSteerAngle,
      steerSpeed: this.config.steerSpeed,
      steerSpeedReduction: this.config.steerSpeedReduction,
    });

    // 7. Aerodynamics
    this.aero = new AerodynamicsSystem({
      downforceFront100Kmh: this.config.aeroDownforceFront,
      downforceRear100Kmh: this.config.aeroDownforceRear,
      dragCoeff: this.config.aeroDragCoeff,
      copPitchSensitivity: this.config.aeroCopPitchSensitivity,
      groundEffectUnderbody: this.config.groundEffectUnderbody,
      groundEffectMaxDownforce: this.config.groundEffectMaxDownforce,
      diffuserStallHeight: this.config.aeroDiffuserStallHeight,
      drsEnabled: this.config.drsEnabled,
      drsDragReduction: this.config.drsDragReduction,
      drsDownforceReduction: this.config.drsDownforceReduction,
      airbrakeEnabled: this.config.airbrakeEnabled,
    });

    this.telemetry = new TelemetrySystem();
  }

  public setConfig(newConfig: VehicleConfig) {
    this.config = { ...newConfig };
    this.powertrain.config = {
      maxTorque: newConfig.maxTorque,
      idleRpm: newConfig.idleRpm,
      maxRpm: newConfig.maxRpm,
      revLimiterRpm: newConfig.revLimiterRpm,
      flywheelInertia: newConfig.flywheelInertia,
      engineBrakingTorque: newConfig.engineBrakingTorque,
      clutchBiteRate: newConfig.clutchBiteRate,
      turboBoostMaxPsi: newConfig.turboBoostMaxPsi,
      turboSpoolRate: newConfig.turboSpoolRate,
      wastegatePressurePsi: newConfig.wastegatePressurePsi,
      reverseRatio: newConfig.reverseRatio,
      forwardGearRatios: newConfig.forwardGearRatios,
      gearRatios: newConfig.gearRatios,
      finalDriveRatio: newConfig.finalDriveRatio,
      maxClutchTorque: newConfig.maxClutchTorque,
      transmissionEfficiency: newConfig.transmissionEfficiency,
      autoBlipDownshift: newConfig.autoBlipDownshift,
    };
    this.differential.config = {
      type: newConfig.differentialType,
      powerRamp: newConfig.diffPowerRamp,
      coastRamp: newConfig.diffCoastRamp,
      preloadTorque: newConfig.diffPreloadTorque,
      drivetrain: newConfig.drivetrain,
      frontTorqueRatio: (newConfig as any).centerFrontTorqueRatio,
    };
    this.brakes.config = {
      maxBrakeTorque: newConfig.brakeForce,
      handbrakeTorque: newConfig.handbrakeForce,
      frontBias: newConfig.brakeBiasFront,
    };
    this.driverAids.config = {
      absMode: newConfig.absMode,
      tcsMode: newConfig.tcsMode,
      wheelbase: newConfig.wheelbase,
      trackWidth: newConfig.trackWidth,
      ackermannRatio: newConfig.ackermannRatio,
      maxSteerAngle: newConfig.maxSteerAngle,
      steerSpeed: newConfig.steerSpeed,
      steerSpeedReduction: newConfig.steerSpeedReduction,
    };
    this.aero.config = {
      downforceFront100Kmh: newConfig.aeroDownforceFront,
      downforceRear100Kmh: newConfig.aeroDownforceRear,
      dragCoeff: newConfig.aeroDragCoeff,
      copPitchSensitivity: newConfig.aeroCopPitchSensitivity,
      groundEffectUnderbody: newConfig.groundEffectUnderbody,
      groundEffectMaxDownforce: newConfig.groundEffectMaxDownforce,
      diffuserStallHeight: newConfig.aeroDiffuserStallHeight,
      drsEnabled: newConfig.drsEnabled,
      drsDragReduction: newConfig.drsDragReduction,
      drsDownforceReduction: newConfig.drsDownforceReduction,
      airbrakeEnabled: newConfig.airbrakeEnabled,
    };

    // Update wheel tire configs
    for (let i = 0; i < 4; i++) {
      const isFront = i < 2;
      (this.wheels[i] as any).radius = newConfig.wheelRadius;
      (this.wheels[i] as any).inertia = newConfig.wheelInertia;
      this.wheels[i].tireConfig = {
        baseGrip: isFront ? newConfig.tireGripFront : newConfig.tireGripRear,
        stiffnessB: newConfig.tireStiffness,
        loadSensitivity: newConfig.tireLoadSensitivity,
        slideFrictionMultiplier: newConfig.slideFrictionMultiplier,
        relaxationLength: newConfig.relaxationLength,
        pneumaticTrailMax: newConfig.tirePneumaticTrailMax,
        camberStiffness: 85,
        optimalTemp: newConfig.optimalTireTemp,
        basePressurePsi: newConfig.tireBasePressure,
      };
    }
  }

  public reset(x: number = 0, z: number = 0, yaw: number = 0) {
    const H = this.config.centerOfGravityHeight;
    this.rigidBody.position = PhysicsMath.vec3(x, H + 0.35, z);
    this.rigidBody.velocity = PhysicsMath.vec3(0, 0, 0);
    this.rigidBody.angularVelocity = PhysicsMath.vec3(0, 0, 0);
    this.rigidBody.orientation = PhysicsMath.quatFromEuler(0, yaw, 0);
    this.rigidBody.clearForces();

    this.wheels.forEach((w) => w.reset(0));
    this.powertrain.reset();
    this.driverAids.reset();
    this.telemetry.reset();
    this.totalSimTime = 0;
    this.smoothedAx = 0;
    this.smoothedAy = 0;
    this.smoothedAz = 0;
    this.exhaustFlameTimer = 0;
  }

  public triggerClutchKick() {
    this.powertrain.triggerClutchKick();
  }

  public toggleDrs() {
    this.aero.toggleDrs();
  }

  /**
   * Get 4 suspension top-mount hardpoints in body local space
   */
  public getHardpointsBody(): [Vec3, Vec3, Vec3, Vec3] {
    const W = this.config.trackWidth * 0.5;
    const L = this.config.wheelbase;
    const frontDist = L * (1.0 - this.config.weightDistributionFront);
    const rearDist = L * this.config.weightDistributionFront;

    return [
      PhysicsMath.vec3(-W, 0, frontDist), // FL
      PhysicsMath.vec3(W, 0, frontDist),  // FR
      PhysicsMath.vec3(-W, 0, -rearDist), // RL
      PhysicsMath.vec3(W, 0, -rearDist),  // RR
    ];
  }

  /**
   * Step the entire 14-DOF vehicle simulation for fixed step dt
   */
  public step(inputs: ControlInputs, dt: number) {
    if (dt <= 0) return;
    this.totalSimTime += dt;

    // Shift handling
    if (inputs.shiftUp) this.powertrain.shiftUp();
    if (inputs.shiftDown) this.powertrain.shiftDown();

    const localVel = this.rigidBody.getLocalVelocity();
    const localW = this.rigidBody.getLocalAngularVelocity();
    const euler = this.rigidBody.getEuler();
    const forwardSpeed = localVel.z;
    const speedMs = PhysicsMath.vec3Length(this.rigidBody.velocity);

    // 1. Steering kinematics with Ackermann geometry
    const steerOut = this.driverAids.updateSteering(inputs.steer, forwardSpeed, dt);
    this.wheels[0].steerAngle = steerOut.steerFL;
    this.wheels[1].steerAngle = steerOut.steerFR;
    const meanFrontSteer = (steerOut.steerFL + steerOut.steerFR) * 0.5;
    const rearMax = (((this.config as any).rearSteerMaxDeg ?? 0) * Math.PI) / 180;
    const rearTransition = Math.max(1, (this.config as any).rearSteerTransitionSpeedMs ?? 20);
    const speedAbs = Math.abs(forwardSpeed);
    const phase = PhysicsMath.clamp((speedAbs - (rearTransition - 5)) / 10, 0, 1);
    const lowSpeedRear = -Math.sign(meanFrontSteer) * Math.min(Math.abs(meanFrontSteer) * 0.35, rearMax);
    const highSpeedRear = Math.sign(meanFrontSteer) * Math.min(Math.abs(meanFrontSteer) * 0.18, rearMax);
    const rearSteer = lowSpeedRear + (highSpeedRear - lowSpeedRear) * phase;
    this.wheels[2].steerAngle = rearSteer;
    this.wheels[3].steerAngle = rearSteer;

    // 2. Suspension ground clearance & solve 4-corner displacements and normal loads
    const hardpointsBody = this.getHardpointsBody();

    const cornerCfgFront: SuspensionCornerConfig = {
      restLength: this.config.suspensionRestLength,
      springStiffness: this.config.suspensionStiffness * 1.05,
      dampingLowSpeed: this.config.suspensionDampingLowSpeed,
      dampingHighSpeed: this.config.suspensionDampingHighSpeed,
      dampingRebound: this.config.suspensionReboundDamping,
      bumpStopStiffness: this.config.bumpStopStiffness,
      bumpStopThreshold: this.config.bumpStopTravelThreshold,
      maxDroop: 0.12,
      maxBump: 0.14,
      staticCamberDeg: this.config.camberStaticFront,
      camberGainDegPerMeter: this.config.camberGain,
      antiDiveSquatRatio: this.config.antiDiveFront,
    };

    const cornerCfgRear: SuspensionCornerConfig = {
      restLength: this.config.suspensionRestLength,
      springStiffness: this.config.suspensionStiffness * 0.95,
      dampingLowSpeed: this.config.suspensionDampingLowSpeed,
      dampingHighSpeed: this.config.suspensionDampingHighSpeed,
      dampingRebound: this.config.suspensionReboundDamping,
      bumpStopStiffness: this.config.bumpStopStiffness,
      bumpStopThreshold: this.config.bumpStopTravelThreshold,
      maxDroop: 0.12,
      maxBump: 0.14,
      staticCamberDeg: this.config.camberStaticRear,
      camberGainDegPerMeter: this.config.camberGain,
      antiDiveSquatRatio: this.config.antiSquatRear,
    };

    this.suspension.update(
      hardpointsBody,
      this.rigidBody.position,
      this.rigidBody.orientation,
      this.rigidBody.velocity,
      this.rigidBody.angularVelocity,
      (x, z) => this.surfaceProvider.sampleSurface(x, z),
      [cornerCfgFront, cornerCfgFront, cornerCfgRear, cornerCfgRear],
      this.config.rollStiffnessFront,
      this.config.rollStiffnessRear,
      this.config.antiRollCrossCoupling,
      this.config.wheelRadius,
      this.config.tireVerticalStiffness,
      dt
    );

    // 3. Aerodynamics (Front & Rear Downforce, Drag, Diffuser Suction)
    const minRideHeight = Math.min(...this.suspension.states.map((s) => this.config.suspensionRestLength - s.displacement));
    const aeroOut = this.aero.calculateAeroForces(
      localVel,
      euler.pitch,
      minRideHeight,
      inputs.brake,
      this.config.wheelbase
    );

    // Apply aero forces to rigid body
    this.rigidBody.addBodyForceAtPoint(aeroOut.frontAeroForce, aeroOut.frontPointBody);
    this.rigidBody.addBodyForceAtPoint(aeroOut.rearAeroForce, aeroOut.rearPointBody);
    this.rigidBody.addBodyForceAtPoint(aeroOut.diffuserAeroForce, aeroOut.diffuserPointBody);
    this.rigidBody.addBodyForceAtPoint(aeroOut.dragForce, PhysicsMath.vec3(0, 0, 0));

    // 4. TCS Throttle Reduction
    const drivenSlips =
      this.config.drivetrain === 'FWD'
        ? [this.wheels[0].relaxationSlipRatio, this.wheels[1].relaxationSlipRatio]
        : this.config.drivetrain === 'RWD'
        ? [this.wheels[2].relaxationSlipRatio, this.wheels[3].relaxationSlipRatio]
        : this.wheels.map((w) => w.relaxationSlipRatio);

    const tcsResult = this.driverAids.updateTCS(drivenSlips, dt);
    const effectiveThrottle = inputs.throttle * tcsResult.throttleMultiplier;

    // 5. Powertrain & Differential Torque Path
    const wheelOmegas: [number, number, number, number] = [
      this.wheels[0].angularVelocity,
      this.wheels[1].angularVelocity,
      this.wheels[2].angularVelocity,
      this.wheels[3].angularVelocity,
    ];

    // Driven axle speed
    const drivenOmega =
      this.config.drivetrain === 'FWD'
        ? (wheelOmegas[0] + wheelOmegas[1]) * 0.5
        : (wheelOmegas[2] + wheelOmegas[3]) * 0.5;

    const powertrainOut = this.powertrain.update(effectiveThrottle, drivenOmega, dt);

    const diffOut = this.differential.distributeTorque(powertrainOut.driveshaftTorque, wheelOmegas);

    // 6. Brakes & ABS Controller
    const wheelSlips: [number, number, number, number] = [
      this.wheels[0].relaxationSlipRatio,
      this.wheels[1].relaxationSlipRatio,
      this.wheels[2].relaxationSlipRatio,
      this.wheels[3].relaxationSlipRatio,
    ];

    const absModulators = this.driverAids.updateABS(
      wheelSlips,
      wheelOmegas,
      speedMs,
      inputs.brake > 0.05,
      dt
    );
    this.brakes.pressureModulators = absModulators;

    const brakeTorques = this.brakes.calculateBrakeTorques(inputs.brake, inputs.handbrake);

    // 7. Solve 4 Wheels & Apply Contact Forces to Rigid Body
    let totalAligningTorque = 0;

    for (let i = 0; i < 4; i++) {
      const wheel = this.wheels[i];
      const suspState = this.suspension.states[i];
      const hpBody = hardpointsBody[i];

      // Ground velocity at wheel contact patch expressed in body coords
      const contactPointBody = PhysicsMath.vec3(hpBody.x, -this.config.centerOfGravityHeight, hpBody.z);
      const vContactBody = this.rigidBody.getPointVelocityBody(contactPointBody);

      // Rotate velocity into wheel heading coordinate frame (steer angle about Y)
      const steer = wheel.steerAngle;
      const cosS = Math.cos(steer);
      const sinS = Math.sin(steer);

      // vx_wheel (longitudinal in wheel rolling direction), vy_wheel (lateral to the right of wheel)
      const vxWheel = vContactBody.x * sinS + vContactBody.z * cosS;
      const vyWheel = vContactBody.x * cosS - vContactBody.z * sinS;

      // Sample local surface friction and properties
      const contactWorld = suspState.contactPointWorld;
      const surface = this.surfaceProvider.sampleSurface(contactWorld.x, contactWorld.z);

      // Step wheel rotational dynamics and compute tire forces (Fx, Fy)
      const tireOut = wheel.update(
        vxWheel,
        vyWheel,
        suspState.forceNorm,
        suspState.dynamicCamberDeg,
        diffOut.wheelTorques[i],
        brakeTorques.hydraulicTorques[i],
        brakeTorques.handbrakeTorques[i],
        surface.friction * this.config.ambientSurfaceFrictionMultiplier,
        surface.rollingResistance,
        dt
      );

      totalAligningTorque += tireOut.aligningTorque;

      if (!suspState.isAirborne && suspState.forceNorm > 0) {
        // Convert tire planar forces (Fx along wheel heading, Fy lateral to wheel) back into body frame
        const fxBody = tireOut.fy * cosS + tireOut.fx * sinS;
        const fzBody = -tireOut.fy * sinS + tireOut.fx * cosS;
        const fyBody = suspState.forceNorm; // Normal upward reaction from suspension

        const forceBody = PhysicsMath.vec3(fxBody, fyBody, fzBody);
        this.rigidBody.addBodyForceAtPoint(forceBody, contactPointBody);

        // Apply self-aligning torque Mz to chassis
        this.rigidBody.addBodyTorque(PhysicsMath.vec3(0, tireOut.aligningTorque, 0));
      }
    }

    // 8. Apply Gravity in World Frame
    const gravityForceWorld = PhysicsMath.vec3(0, -this.config.mass * 9.81, 0);
    this.rigidBody.addWorldForce(gravityForceWorld);

    // 9. Integrate 6-DOF Rigid Body Equations of Motion
    this.rigidBody.integrate(dt);

    // 10. Update Telemetry, Performance Timers & G-Forces
    const rawAx = this.rigidBody.acceleration.x / 9.81;
    const rawAy = this.rigidBody.acceleration.y / 9.81;
    const rawAz = this.rigidBody.acceleration.z / 9.81;

    // Smooth G-forces
    const gAlpha = Math.min(1.0, 15.0 * dt);
    this.smoothedAx += (rawAx - this.smoothedAx) * gAlpha;
    this.smoothedAy += (rawAy - this.smoothedAy) * gAlpha;
    this.smoothedAz += (rawAz - this.smoothedAz) * gAlpha;

    const speedKmh = speedMs * 3.6;
    const speedMph = speedKmh * 0.621371;

    this.telemetry.updateTimersAndGForces(
      speedKmh,
      speedMs,
      this.smoothedAx,
      this.smoothedAz,
      inputs.throttle,
      this.totalSimTime,
      dt
    );

    this.telemetry.updateDriftScore(speedKmh, localVel.x, localVel.z, dt);

    // Exhaust flame timer
    if (this.powertrain.turboBlowOff || (this.powertrain.isRevLimiting && this.powertrain.revCutBounce)) {
      this.exhaustFlameTimer = 0.15;
    } else if (this.exhaustFlameTimer > 0) {
      this.exhaustFlameTimer -= dt;
    }
  }

  /**
   * Export complete read-only VehicleState snapshot for rendering, UI, audio, and tests
   */
  public getState(): VehicleState {
    const pos = this.rigidBody.position;
    const euler = this.rigidBody.getEuler();
    const localVel = this.rigidBody.getLocalVelocity();
    const localW = this.rigidBody.getLocalAngularVelocity();
    const speedMs = PhysicsMath.vec3Length(this.rigidBody.velocity);
    const speedKmh = speedMs * 3.6;
    const speedMph = speedKmh * 0.621371;

    const hardpointsBody = this.getHardpointsBody();

    // Map 4 WheelStates
    const wheelStates: [WheelState, WheelState, WheelState, WheelState] = this.wheels.map((w, idx) => {
      const susp = this.suspension.states[idx];
      const tire = w.lastTireOutput;
      const hp = hardpointsBody[idx];
      const surface = this.surfaceProvider.sampleSurface(susp.contactPointWorld.x, susp.contactPointWorld.z);

      return {
        id: w.id,
        isFront: w.isFront,
        isLeft: w.isLeft,
        localPos: { x: hp.x, y: hp.y, z: hp.z },
        steerAngle: w.steerAngle,
        camberAngleDeg: susp.dynamicCamberDeg,
        rotationAngle: w.rotationAngle,
        angularVelocity: w.angularVelocity,
        suspensionCompression: susp.compressionRatio,
        damperVelocity: susp.velocity,
        verticalTravelM: susp.hubTravelM,
        bumpStopEngaged: susp.bumpStopEngaged,
        suspensionForce: susp.forceNorm,
        slipAngle: w.relaxationSlipAngle,
        slipRatio: w.relaxationSlipRatio,
        pneumaticTrail: tire.pneumaticTrail,
        aligningTorque: tire.aligningTorque,
        sidewallDeflection: tire.sidewallDeflection,
        tireSquishM: tire.tireSquishM,
        isAirborne: susp.isAirborne,
        isSkidding: tire.isSkidding,
        skidIntensity: tire.skidIntensity,
        groundContactPos: {
          x: susp.contactPointWorld.x,
          y: susp.contactPointWorld.y,
          z: susp.contactPointWorld.z,
        },
        temperature: w.temperature,
        pressurePsi: w.pressurePsi,
        tireWearPercent: w.wearPercent,
        grainPercent: 0,
        brakeRotorTemp: w.brakeRotorTemp,
        surfaceType: surface.type,
        surfaceFriction: surface.friction,
        forceVectorLong: tire.fx,
        forceVectorLat: tire.fy,
        forceVectorNorm: susp.forceNorm,
        frictionLimitN: tire.frictionLimit,
        gripUtilization: tire.gripUtilization,
        absActive: this.driverAids.absActive && this.brakes.pressureModulators[idx] < 0.9,
        tcsActive: this.driverAids.tcsActive,
      };
    }) as [WheelState, WheelState, WheelState, WheelState];

    const airborneCount = this.suspension.states.filter((s) => s.isAirborne).length;
    const cgSurface = this.surfaceProvider.sampleSurface(pos.x, pos.z);

    const shiftStage = this.telemetry.getShiftLightStage(
      this.powertrain.engineRpm,
      this.config.maxRpm,
      this.config.revLimiterRpm
    );

    const driftInfo = this.telemetry.updateDriftScore(speedKmh, localVel.x, localVel.z, 0);

    return {
      x: pos.x,
      y: pos.y,
      z: pos.z,
      yaw: euler.yaw,
      pitch: euler.pitch * this.config.bodyPitchMultiplier,
      roll: euler.roll * this.config.bodyRollMultiplier,
      heave: pos.y - (this.config.centerOfGravityHeight + 0.35),
      vx: localVel.x,
      vy: localVel.y,
      vz: localVel.z,
      yawRate: localW.y,
      rollRate: localW.z,
      pitchRate: localW.x,
      speedMs,
      speedKmh,
      speedMph,
      rpm: this.powertrain.engineRpm,
      flywheelRpm: this.powertrain.flywheelRpm,
      gear: this.powertrain.gear,
      isAutomatic: this.powertrain.isAutomatic,
      clutchEngaged: this.powertrain.clutchEngaged,
      clutchPedal: this.powertrain.clutchPedal,
      clutchKickImpulse: this.powertrain.clutchEngaged ? 0 : 1.0,
      isRevLimiting: this.powertrain.isRevLimiting,
      revCutBounce: this.powertrain.revCutBounce,
      engineTorqueDelivered: this.powertrain.deliveredDriveshaftTorque,
      throttle: this.powertrain.engineTorqueOutput > 0 ? 1 : 0,
      brake: 0,
      handbrake: false,
      steerInput: 0,
      actualSteerAngle: this.driverAids.currentCenterSteerAngle,
      turboBoostPsi: this.powertrain.turboBoostPsi,
      turboBlowOff: this.powertrain.turboBlowOff,
      wastegateOpen: this.powertrain.wastegateOpen,
      exhaustFlameIntensity: this.exhaustFlameTimer > 0 ? 1.0 : 0,
      launchControlActive: false,
      shiftLightStage: shiftStage,
      drsActive: this.aero.drsActive,
      airbrakeActive: this.aero.airbrakeActive,
      centerOfPressureShift: 0,
      aeroDownforceTotalN: this.aero.totalDownforceN,
      diffuserRideHeightM: this.aero.diffuserRideHeightM,
      diffuserStalled: this.aero.diffuserStalled,
      steeringRackTorque: 0,
      totalAligningTorque: 0,
      elevationHeight: cgSurface.elevation,
      terrainSlopePitch: cgSurface.slopePitch,
      kerbRumbleIntensity: cgSurface.isKerbRumble ? 0.85 : 0,
      airborneWheelsCount: airborneCount,
      lateralG: this.smoothedAx,
      longitudinalG: this.smoothedAz,
      verticalG: this.smoothedAy,
      gForceHistory: this.telemetry.gForceHistory,
      showForceVectors3D: this.showForceVectors3D,
      driftAngleDeg: driftInfo.driftAngleDeg,
      isDrifting: driftInfo.isDrifting,
      driftScore: this.telemetry.driftScore,
      absActive: this.driverAids.absActive,
      tcsActive: this.driverAids.tcsActive,
      wheels: wheelStates,
      performanceTimer: this.telemetry.performanceTimer,
    };
  }
}