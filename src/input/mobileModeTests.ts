import assert from 'node:assert/strict';
import { shouldAutoEnableMobileMode } from './mobileMode';

const iPhone = shouldAutoEnableMobileMode({
  userAgent: 'Mozilla/5.0 (iPhone; CPU iPhone OS 18_0 like Mac OS X) AppleWebKit Mobile/15E148',
  maxTouchPoints: 5,
  coarsePointer: true,
  viewportWidth: 430,
});
assert.equal(iPhone, true, 'iPhone should auto-enable mobile mode');

const android = shouldAutoEnableMobileMode({
  userAgent: 'Mozilla/5.0 (Linux; Android 15; Pixel 9 Pro) AppleWebKit Chrome Mobile Safari',
  maxTouchPoints: 5,
  coarsePointer: true,
  viewportWidth: 412,
});
assert.equal(android, true, 'Android phones should auto-enable mobile mode');

const ipadDesktopUa = shouldAutoEnableMobileMode({
  userAgent: 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15) AppleWebKit Safari',
  maxTouchPoints: 5,
  coarsePointer: true,
  viewportWidth: 1024,
});
assert.equal(ipadDesktopUa, true, 'iPad desktop-style user agent should auto-enable mobile mode');

const narrowDesktop = shouldAutoEnableMobileMode({
  userAgent: 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit Chrome Safari',
  maxTouchPoints: 0,
  coarsePointer: false,
  viewportWidth: 800,
});
assert.equal(narrowDesktop, false, 'A narrow desktop window must not enable mobile mode');

const wideTouchLaptop = shouldAutoEnableMobileMode({
  userAgent: 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit Chrome Safari',
  maxTouchPoints: 10,
  coarsePointer: true,
  viewportWidth: 1440,
});
assert.equal(wideTouchLaptop, false, 'A wide touchscreen laptop should remain in desktop mode');

const compactTouchDevice = shouldAutoEnableMobileMode({
  userAgent: 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit Chrome Safari',
  maxTouchPoints: 5,
  coarsePointer: true,
  viewportWidth: 900,
});
assert.equal(compactTouchDevice, true, 'A compact coarse-pointer touch device should enable mobile mode');

console.log('Mobile mode detection tests passed.');
