import { PhysicsMath } from './math/PhysicsMath';

/**
 * Keyboard/touch steering is binary hardware pretending to be a steering wheel.
 * Feeding +/-1 straight into the rack asks for parking-lot lock at road speed,
 * which can demand impossible lateral acceleration and manufacture understeer.
 *
 * This adapter preserves full lock at crawl/parking speeds, then progressively
 * limits the digital command with speed while rate-limiting both turn-in and
 * return-to-center. Analog/wheel inputs should bypass this helper entirely.
 */
export function digitalSteeringLimitForSpeed(speedMs: number): number {
  const speedKmh = Math.abs(speedMs) * 3.6;

  if (speedKmh <= 8) return 1.0;
  if (speedKmh <= 15) return PhysicsMath.lerp(1.0, 0.72, (speedKmh - 8) / 7);
  if (speedKmh <= 30) return PhysicsMath.lerp(0.72, 0.48, (speedKmh - 15) / 15);
  if (speedKmh <= 60) return PhysicsMath.lerp(0.48, 0.27, (speedKmh - 30) / 30);
  if (speedKmh <= 100) return PhysicsMath.lerp(0.27, 0.18, (speedKmh - 60) / 40);
  if (speedKmh <= 160) return PhysicsMath.lerp(0.18, 0.12, (speedKmh - 100) / 60);
  return 0.12;
}

export function updateDigitalSteeringInput(
  currentInput: number,
  direction: -1 | 0 | 1,
  speedMs: number,
  dt: number
): number {
  if (dt <= 0) return PhysicsMath.clamp(currentInput, -1, 1);

  const limit = digitalSteeringLimitForSpeed(speedMs);
  const target = direction * limit;

  // Turn-in is deliberately slower than recentering. At 30 km/h the digital
  // command reaches its ~0.48 cap in ~0.18 s instead of snapping to full rack in
  // a single frame. Returning the wheel is quicker so transitions stay natural.
  const movingTowardCenter =
    direction === 0 ||
    (Math.sign(currentInput) !== 0 && Math.sign(target) !== Math.sign(currentInput));
  const ratePerSecond = movingTowardCenter ? 4.8 : 2.7;
  const maxStep = ratePerSecond * dt;
  const error = target - currentInput;

  if (Math.abs(error) <= maxStep) return target;
  return PhysicsMath.clamp(currentInput + Math.sign(error) * maxStep, -limit, limit);
}
