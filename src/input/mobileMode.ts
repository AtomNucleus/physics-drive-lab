export interface MobileModeSignals {
  userAgent: string;
  maxTouchPoints: number;
  coarsePointer: boolean;
  viewportWidth: number;
}

const MOBILE_USER_AGENT = /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini|Mobile/i;

/**
 * Decide whether the touch-first driving UI should enable itself automatically.
 * Desktop remains the default. Phones, Android devices, and iPads (including the
 * desktop-style iPad user agent) opt into mobile mode automatically.
 */
export function shouldAutoEnableMobileMode(signals: MobileModeSignals): boolean {
  const { userAgent, maxTouchPoints, coarsePointer, viewportWidth } = signals;
  const mobileUserAgent = MOBILE_USER_AGENT.test(userAgent);
  const ipadDesktopUserAgent = /Macintosh/i.test(userAgent) && maxTouchPoints > 1;
  const compactTouchDevice = maxTouchPoints > 0 && coarsePointer && viewportWidth <= 1180;

  return mobileUserAgent || ipadDesktopUserAgent || compactTouchDevice;
}

export function readBrowserMobileModeSignals(): MobileModeSignals {
  if (typeof window === 'undefined' || typeof navigator === 'undefined') {
    return {
      userAgent: '',
      maxTouchPoints: 0,
      coarsePointer: false,
      viewportWidth: Number.POSITIVE_INFINITY,
    };
  }

  return {
    userAgent: navigator.userAgent || '',
    maxTouchPoints: navigator.maxTouchPoints || 0,
    coarsePointer: window.matchMedia?.('(pointer: coarse)').matches ?? false,
    viewportWidth: window.innerWidth,
  };
}
