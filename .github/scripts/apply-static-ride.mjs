import fs from 'node:fs';

const path = 'src/physics/Vehicle.ts';
let text = fs.readFileSync(path, 'utf8');

const oldImport = "import { SuspensionSystem, SuspensionCornerConfig } from './Suspension';";
const newImport = "import { SuspensionSystem, SuspensionCornerConfig, progressiveSpringIncrement, bumpStopForceForDisplacement } from './Suspension';";
if (text.includes(oldImport)) text = text.replace(oldImport, newImport);

const oldReset = `  public reset(x: number = 0, z: number = 0, yaw: number = 0) {
    const H = this.config.centerOfGravityHeight;
    this.rigidBody.position = PhysicsMath.vec3(x, H + 0.35, z);
    this.rigidBody.velocity = PhysicsMath.vec3(0, 0, 0);
    this.rigidBody.angularVelocity = PhysicsMath.vec3(0, 0, 0);
    this.rigidBody.orientation = PhysicsMath.quatFromEuler(0, yaw, 0);
    this.rigidBody.clearForces();
`;

const newReset = `  public reset(x: number = 0, z: number = 0, yaw: number = 0) {
    const H = this.config.centerOfGravityHeight;

    // Start at the static spring/tire equilibrium so reset does not inject a fake
    // vertical impact into the new unsprung-mass system.
    const maxDroop = 0.12;
    const maxBump = 0.14;
    const tireK = Math.max(1000, this.config.tireVerticalStiffness);
    const surfaceY = this.surfaceProvider.sampleSurface(x, z).elevation;
    const staticHardpointY = (targetLoadN: number, springRate: number) => {
      let low = -maxDroop;
      let high = maxBump;
      const preload = maxDroop * 0.78 * Math.max(0, springRate);
      for (let iteration = 0; iteration < 32; iteration++) {
        const displacement = (low + high) * 0.5;
        const springForce = Math.max(0, preload + progressiveSpringIncrement(displacement, springRate, maxBump, 0.65));
        const bumpStopForce = bumpStopForceForDisplacement(displacement, maxBump, this.config.bumpStopTravelThreshold, this.config.bumpStopStiffness);
        if (springForce + bumpStopForce < targetLoadN) low = displacement;
        else high = displacement;
      }
      const displacement = (low + high) * 0.5;
      const tireCompression = targetLoadN / tireK;
      const hubY = surfaceY + this.config.wheelRadius - tireCompression;
      return hubY + maxDroop + this.config.suspensionRestLength - displacement;
    };
    const frontTargetLoad = this.config.mass * 9.81 * this.config.weightDistributionFront * 0.5;
    const rearTargetLoad = this.config.mass * 9.81 * (1 - this.config.weightDistributionFront) * 0.5;
    const frontHardpointY = staticHardpointY(frontTargetLoad, this.config.suspensionStiffness * 1.05);
    const rearHardpointY = staticHardpointY(rearTargetLoad, this.config.suspensionStiffness * 0.95);
    const wheelbase = Math.max(0.001, this.config.wheelbase);
    const frontDist = wheelbase * (1 - this.config.weightDistributionFront);
    const rearDist = wheelbase * this.config.weightDistributionFront;
    const sinStaticPitch = PhysicsMath.clamp((rearHardpointY - frontHardpointY) / wheelbase, -0.15, 0.15);
    const staticPitch = Math.asin(sinStaticPitch);
    const bodyYFront = frontHardpointY + frontDist * sinStaticPitch;
    const bodyYRear = rearHardpointY - rearDist * sinStaticPitch;
    const staticBodyY = (bodyYFront + bodyYRear) * 0.5;

    this.rigidBody.position = PhysicsMath.vec3(x, staticBodyY, z);
    this.rigidBody.velocity = PhysicsMath.vec3(0, 0, 0);
    this.rigidBody.angularVelocity = PhysicsMath.vec3(0, 0, 0);
    this.rigidBody.orientation = PhysicsMath.quatFromEuler(staticPitch, yaw, 0);
    this.rigidBody.clearForces();
`;

if (text.includes(oldReset)) {
  text = text.replace(oldReset, newReset);
  fs.writeFileSync(path, text);
} else if (!text.includes(newReset)) {
  throw new Error('Vehicle.reset anchor not found');
}
