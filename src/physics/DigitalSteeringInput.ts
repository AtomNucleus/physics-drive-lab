import { PhysicsMath } from './math/PhysicsMath';

/**
 * Keyboard/touch inputs are binary, so steering-in/reversal gets a small driver-input
 * slew to approximate hands moving a steering wheel. The physical rack owns the real
 * steering inertia, damping, rate and road-force response, so this helper must not
 * become a second steering system or remove rack authority.
 *
 * Releasing the control drops the driver's target to zero immediately. The road
 * wheels still unwind physically through the rack instead of snapping to center.
 */
export function updateDigitalSteeringInput(
  currentInput: number,
  direction: -1 | 0 | 1,
  _speedMs: number,
  dt: number
): number {
  if (dt <= 0) return PhysicsMath.clamp(currentInput, -1, 1);
  if (direction === 0) return 0;

  const target = direction;
  const reversingDirection =
    Math.sign(currentInput) !== 0 && Math.sign(target) !== Math.sign(currentInput);
  const ratePerSecond = reversingDirection ? 7.0 : 4.8;
  const maxStep = ratePerSecond * dt;
  const error = target - currentInput;

  if (Math.abs(error) <= maxStep) return target;
  return PhysicsMath.clamp(currentInput + Math.sign(error) * maxStep, -1, 1);
}
