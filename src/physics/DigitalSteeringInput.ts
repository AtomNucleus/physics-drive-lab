import { PhysicsMath } from './math/PhysicsMath';

/**
 * Keyboard/touch inputs are binary, so steering-in/reversal still gets a small
 * driver-input slew to approximate hands moving a steering wheel. The physical rack
 * now owns the actual steering inertia, damping, rate and road-force response, so
 * this helper must not become a second steering system or remove rack authority.
 *
 * Releasing the key/button is intentionally different: the driver request drops to
 * zero immediately. The road wheels do NOT snap to center because the physical rack
 * retains angle/rate/inertia and must unwind through caster, tire aligning torque,
 * steering damping and EPS/column forces. Keeping an artificial release ramp here
 * caused a double-lag and a measurable opposite-yaw overshoot during an oversteer
 * catch after the physical rack was introduced.
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

  // A driver can throw the wheel back through center faster than they normally
  // wind steering into a corner. This is input-device emulation only: it does not
  // inspect yaw, sideslip, tire state, or vehicle motion and never adds forces.
  const reversingDirection =
    Math.sign(currentInput) !== 0 && Math.sign(target) !== Math.sign(currentInput);
  const ratePerSecond = reversingDirection ? 7.0 : 4.8;
  const maxStep = ratePerSecond * dt;
  const error = target - currentInput;

  if (Math.abs(error) <= maxStep) return target;
  return PhysicsMath.clamp(currentInput + Math.sign(error) * maxStep, -1, 1);
}
