export type MobileSteeringDirection = 'left' | 'right';
export type MobileSteeringAction = 'steerLeft' | 'steerRight';

/**
 * VehiclePhysicsEngine uses the opposite steering sign from the touch arrow labels.
 * Keep that coordinate-system conversion in one place so the visible arrow always
 * matches the direction the car actually turns.
 */
export function mapMobileSteeringDirection(direction: MobileSteeringDirection): MobileSteeringAction {
  return direction === 'left' ? 'steerRight' : 'steerLeft';
}
