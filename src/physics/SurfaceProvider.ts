import { SurfaceType } from '../types';
import { Vec3, PhysicsMath } from './math/PhysicsMath';

export interface SurfaceSample {
  elevation: number;
  normal: Vec3;
  slopePitch: number;
  slopeRoll: number;
  type: SurfaceType;
  friction: number;
  rollingResistance: number;
  wetness: number;
  isKerbRumble: boolean;
}

export interface ISurfaceProvider {
  sampleSurface(x: number, z: number): SurfaceSample;
}

export class ProvingGroundSurfaceProvider implements ISurfaceProvider {
  public sampleSurface(x: number, z: number): SurfaceSample {
    let elevation = 0;
    let slopePitch = 0;
    let slopeRoll = 0;
    let type: SurfaceType = 'asphalt';
    let friction = 1.0;
    let rollingResistance = 0.015;
    let wetness = 0.0;
    let isKerbRumble = false;

    // 1. Proving Ground Test Features:
    // A. 10% Incline Hill Climb Test Section (X in [50, 75], Z in [200, 450])
    if (x >= 50 && x <= 75 && z >= 200 && z <= 450) {
      const inclinePhase = (z - 200) / 250; // 0 to 1
      elevation = inclinePhase * 25.0; // 25m rise over 250m = 10% grade
      slopePitch = 0.10; // ~5.7 deg pitch
      friction = 1.0;
      type = 'asphalt';
    }

    // B. High-Speed Crest (Z in [60, 260], |X| <= 22)
    else if (Math.abs(x) < 22 && z > 60 && z < 260) {
      const crestPhase = (z - 60) / 200; // 0..1
      elevation = Math.sin(crestPhase * Math.PI) * 1.65;
      slopePitch = Math.cos(crestPhase * Math.PI) * 0.026;
    }

    // C. High-Speed Dip / Compression Bowl (Z in [-260, -60], |X| <= 22)
    else if (Math.abs(x) < 22 && z < -60 && z > -260) {
      const dipPhase = (z - (-60)) / -200; // 0..1
      elevation = -Math.sin(dipPhase * Math.PI) * 1.15;
      slopePitch = -Math.cos(dipPhase * Math.PI) * 0.022;
    }

    // D. Proving Ground Single Bump & Alternating Kerb Bumps Section (X in [-60, -35], Z in [100, 250])
    else if (x >= -60 && x <= -35 && z >= 100 && z <= 250) {
      // Single bump at Z = 120
      if (Math.abs(z - 120) < 1.5) {
        elevation = Math.cos(((z - 120) / 1.5) * (Math.PI / 2)) * 0.06; // 6cm smooth speed bump
      }
      // Alternating washboard kerb bumps between Z = 160 and 230
      if (z >= 160 && z <= 230) {
        const bumpWave = Math.sin((z - 160) * 1.8) * Math.sin((x + 47.5) * 1.2);
        elevation = Math.max(0, bumpWave * 0.04);
        isKerbRumble = true;
        friction = 0.95;
        type = 'kerb';
      }
    }

    // E. Banked High-Speed Oval Corner Arc (Radius ~140m centered at 0, 350)
    const distToNorthTurn = Math.hypot(x, z - 350);
    if (distToNorthTurn >= 110 && distToNorthTurn <= 165 && z >= 350) {
      const radialOffset = (distToNorthTurn - 110) / 55; // 0 to 1 from inside to outside
      // Bank angle rises to 12 degrees (tan ~ 0.21)
      elevation = radialOffset * 2.8;
      slopeRoll = 0.20 * Math.sign(x);
      friction = 1.05;
    }

    // 2. Specialized Arena Surfaces
    // Wet / Polished Concrete Skidpad Arena (Center: 85, -60, Radius: 76)
    const distToWet = Math.hypot(x - 85, z - (-60));
    if (distToWet <= 76) {
      type = 'wet';
      friction = 0.42;
      rollingResistance = 0.012;
      wetness = 0.85;
      return {
        elevation,
        normal: this.calculateNormal(slopePitch, slopeRoll),
        slopePitch,
        slopeRoll,
        type,
        friction,
        rollingResistance,
        wetness,
        isKerbRumble: false,
      };
    }

    // Dry High-Grip Rubbered Skidpad Arena (Center: -85, 60, Radius: 76)
    const distToDry = Math.hypot(x - (-85), z - 60);
    if (distToDry <= 76) {
      type = 'racing_line';
      friction = 1.14;
      rollingResistance = 0.018;
      return {
        elevation,
        normal: this.calculateNormal(slopePitch, slopeRoll),
        slopePitch,
        slopeRoll,
        type,
        friction,
        rollingResistance,
        wetness: 0,
        isKerbRumble: false,
      };
    }

    // 3. Main Runway & Drag Strip (X in [-18, 18], Z in [-510, 510])
    if (Math.abs(z) <= 510 && !isKerbRumble) {
      const absX = Math.abs(x);
      if (absX <= 6.5) {
        type = 'racing_line';
        friction = 1.10;
        rollingResistance = 0.016;
      } else if (absX <= 17.5) {
        type = 'asphalt';
        friction = 1.0;
        rollingResistance = 0.015;
      } else if (absX <= 20.0) {
        // Raised 3D Kerb rumble strip
        elevation += 0.045;
        type = 'kerb';
        friction = 0.88;
        rollingResistance = 0.024;
        isKerbRumble = true;
      } else if (absX <= 24.5) {
        type = 'marbles';
        friction = 0.72;
        rollingResistance = 0.035;
      } else {
        type = 'gravel';
        friction = 0.55;
        rollingResistance = 0.075;
      }
    }

    return {
      elevation,
      normal: this.calculateNormal(slopePitch, slopeRoll),
      slopePitch,
      slopeRoll,
      type,
      friction,
      rollingResistance,
      wetness,
      isKerbRumble,
    };
  }

  private calculateNormal(pitch: number, roll: number): Vec3 {
    // Normal vector from slope gradients
    const nx = -Math.sin(roll);
    const nz = -Math.sin(pitch);
    const ny = Math.sqrt(Math.max(0, 1 - nx * nx - nz * nz));
    return PhysicsMath.vec3Normalize(PhysicsMath.vec3(nx, ny, nz));
  }
}
