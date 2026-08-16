import React from 'react';
import { CameraMode } from '../types';
import { VEHICLE_PRESETS } from '../physics/vehiclePresets';
import { readBrowserMobileModeSignals, shouldAutoEnableMobileMode } from '../input/mobileMode';
import {
  Activity,
  Camera,
  Car,
  ChevronDown,
  ChevronUp,
  Cpu,
  HelpCircle,
  RotateCcw,
  Sliders,
  Smartphone,
  Sparkles,
  Volume2,
  VolumeX,
} from 'lucide-react';

interface ControlsOverlayProps {
  cameraMode: CameraMode;
  onNextCamera: () => void;
  onReset: () => void;
  onClearSkidMarks: () => void;
  isMuted: boolean;
  onToggleMute: () => void;
  onOpenTuning: () => void;
  onOpenTestRunner?: () => void;
  showTelemetry: boolean;
  onToggleTelemetry: () => void;
  activePresetKey: string;
  onSelectPreset: (key: string) => void;
  activeKeys: { [key: string]: boolean };
  onTouchInput: (action: 'throttle' | 'brake' | 'steerLeft' | 'steerRight' | 'handbrake', active: boolean) => void;
}

type TouchAction = 'throttle' | 'brake' | 'steerLeft' | 'steerRight' | 'handbrake';

const TOUCH_ACTIONS: TouchAction[] = ['throttle', 'brake', 'steerLeft', 'steerRight', 'handbrake'];

export const ControlsOverlay: React.FC<ControlsOverlayProps> = ({
  cameraMode,
  onNextCamera,
  onReset,
  onClearSkidMarks,
  isMuted,
  onToggleMute,
  onOpenTuning,
  onOpenTestRunner,
  showTelemetry,
  onToggleTelemetry,
  activePresetKey,
  activeKeys,
  onTouchInput,
}) => {
  const [toolbarExpanded, setToolbarExpanded] = React.useState(false);
  const [showHelp, setShowHelp] = React.useState(false);
  const [mobileModeOverride, setMobileModeOverride] = React.useState<boolean | null>(null);
  const [mobileMode, setMobileMode] = React.useState(() => shouldAutoEnableMobileMode(readBrowserMobileModeSignals()));
  const activePreset = VEHICLE_PRESETS[activePresetKey] || VEHICLE_PRESETS.sportGT;

  const isW = activeKeys['KeyW'] || activeKeys['ArrowUp'];
  const isS = activeKeys['KeyS'] || activeKeys['ArrowDown'];
  const isA = activeKeys['KeyA'] || activeKeys['ArrowLeft'];
  const isD = activeKeys['KeyD'] || activeKeys['ArrowRight'];
  const isSpace = activeKeys['Space'];

  React.useEffect(() => {
    const coarsePointer = window.matchMedia('(pointer: coarse)');

    const updateMobileMode = () => {
      const automaticMode = shouldAutoEnableMobileMode(readBrowserMobileModeSignals());
      setMobileMode(mobileModeOverride ?? automaticMode);
    };

    updateMobileMode();
    window.addEventListener('resize', updateMobileMode);
    coarsePointer.addEventListener?.('change', updateMobileMode);

    return () => {
      window.removeEventListener('resize', updateMobileMode);
      coarsePointer.removeEventListener?.('change', updateMobileMode);
    };
  }, [mobileModeOverride]);

  React.useEffect(() => {
    const releaseAllTouchInputs = () => {
      TOUCH_ACTIONS.forEach((action) => onTouchInput(action, false));
    };

    const onVisibilityChange = () => {
      if (document.hidden) releaseAllTouchInputs();
    };

    window.addEventListener('blur', releaseAllTouchInputs);
    document.addEventListener('visibilitychange', onVisibilityChange);

    return () => {
      window.removeEventListener('blur', releaseAllTouchInputs);
      document.removeEventListener('visibilitychange', onVisibilityChange);
      releaseAllTouchInputs();
    };
    // The callback writes into App's persistent input ref; this listener only needs to be installed once.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const toggleMobileMode = () => {
    if (mobileMode) {
      TOUCH_ACTIONS.forEach((action) => onTouchInput(action, false));
    }
    setMobileModeOverride(!mobileMode);
  };

  const topSafeAreaStyle: React.CSSProperties = {
    top: 'max(0.75rem, env(safe-area-inset-top))',
  };

  return (
    <>
      {/* Compact utility bar. Desktop stays unchanged; mobile mode can be enabled manually from the expanded tools. */}
      <div className="absolute left-1/2 z-20 -translate-x-1/2" style={topSafeAreaStyle}>
        <div className="flex items-center gap-1 rounded-2xl border border-slate-800/75 bg-slate-950/80 p-1 shadow-2xl backdrop-blur-xl">
          <button
            id="preset-selector-btn"
            onClick={onOpenTuning}
            className="flex h-8 max-w-32 items-center gap-1.5 rounded-xl px-2 text-[10px] font-bold text-slate-200 hover:bg-slate-800/80"
            title="Vehicle / tuning"
          >
            <span className="h-2 w-2 shrink-0 rounded-full" style={{ backgroundColor: activePreset.color }} />
            <span className="truncate">{activePreset.name}</span>
          </button>

          <button
            id="camera-mode-btn"
            onClick={onNextCamera}
            className="flex h-8 items-center gap-1 rounded-xl px-2 text-[10px] font-semibold text-slate-400 hover:bg-slate-800/80 hover:text-sky-300"
            title="Switch camera (C)"
          >
            <Camera size={14} />
            <span className="hidden capitalize sm:inline">{cameraMode}</span>
          </button>

          <button
            id="toolbar-expand-btn"
            onClick={() => setToolbarExpanded((open) => !open)}
            className="flex h-8 w-8 items-center justify-center rounded-xl text-slate-500 hover:bg-slate-800/80 hover:text-white"
            title={toolbarExpanded ? 'Collapse controls' : 'More controls'}
          >
            {toolbarExpanded ? <ChevronUp size={15} /> : <ChevronDown size={15} />}
          </button>
        </div>

        {toolbarExpanded && (
          <div className="mt-1.5 flex max-w-[calc(100vw-1rem)] flex-wrap items-center justify-center gap-1 rounded-2xl border border-slate-800/75 bg-slate-950/88 p-1 shadow-2xl backdrop-blur-xl">
            <button
              id="tuning-btn"
              onClick={onOpenTuning}
              className="flex h-8 items-center gap-1 rounded-xl px-2 text-[10px] font-semibold text-slate-400 hover:bg-slate-800 hover:text-sky-300"
              title="Physics tuning (P)"
            >
              <Sliders size={14} />
              <span className="hidden sm:inline">Physics</span>
            </button>

            {onOpenTestRunner && (
              <button
                id="physics-tests-btn"
                onClick={onOpenTestRunner}
                className="flex h-8 items-center gap-1 rounded-xl px-2 text-[10px] font-semibold text-sky-300 hover:bg-sky-950/60"
                title="Physics tests (Y)"
              >
                <Cpu size={14} />
                <span className="hidden sm:inline">Tests</span>
              </button>
            )}

            <button
              id="mobile-mode-toggle-btn"
              onClick={toggleMobileMode}
              className={`flex h-8 items-center gap-1 rounded-xl px-2 text-[10px] font-semibold ${
                mobileMode ? 'bg-sky-500/15 text-sky-300' : 'text-slate-500 hover:bg-slate-800 hover:text-white'
              }`}
              title={mobileMode ? 'Disable touch driving controls' : 'Enable touch driving controls'}
            >
              <Smartphone size={14} />
              <span className="hidden sm:inline">Touch</span>
            </button>

            <button
              id="telemetry-toggle-btn"
              onClick={onToggleTelemetry}
              className={`flex h-8 w-8 items-center justify-center rounded-xl ${
                showTelemetry ? 'bg-sky-500/15 text-sky-300' : 'text-slate-500 hover:bg-slate-800 hover:text-white'
              }`}
              title="Toggle detailed telemetry (T)"
            >
              <Activity size={14} />
            </button>

            <button
              id="sound-toggle-btn"
              onClick={onToggleMute}
              className={`flex h-8 w-8 items-center justify-center rounded-xl ${
                isMuted ? 'text-slate-500 hover:bg-slate-800 hover:text-white' : 'bg-emerald-500/10 text-emerald-300'
              }`}
              title="Toggle audio (U)"
            >
              {isMuted ? <VolumeX size={14} /> : <Volume2 size={14} />}
            </button>

            <button
              id="reset-car-btn"
              onClick={onReset}
              className="flex h-8 w-8 items-center justify-center rounded-xl text-slate-500 hover:bg-slate-800 hover:text-amber-300"
              title="Reset car (R)"
            >
              <RotateCcw size={14} />
            </button>

            <button
              id="clear-skids-btn"
              onClick={onClearSkidMarks}
              className="flex h-8 w-8 items-center justify-center rounded-xl text-slate-500 hover:bg-slate-800 hover:text-slate-200"
              title="Clear skid marks"
            >
              <Sparkles size={14} />
            </button>

            <button
              id="help-toggle-btn"
              onClick={() => setShowHelp((open) => !open)}
              className="flex h-8 w-8 items-center justify-center rounded-xl text-slate-500 hover:bg-slate-800 hover:text-white"
              title="Driving controls"
            >
              <HelpCircle size={14} />
            </button>
          </div>
        )}
      </div>

      {showHelp && (
        <div className="absolute left-1/2 top-24 z-30 w-[min(22rem,calc(100vw-1.5rem))] -translate-x-1/2 rounded-2xl border border-slate-700/80 bg-slate-950/94 p-3 text-[10px] text-slate-300 shadow-2xl backdrop-blur-xl">
          <div className="mb-2 flex items-center justify-between border-b border-slate-800 pb-2">
            <span className="flex items-center gap-1.5 font-bold uppercase tracking-wider text-white">
              <Car size={14} className="text-sky-300" />
              {mobileMode ? 'Touch controls' : 'Driving controls'}
            </span>
            <button onClick={() => setShowHelp(false)} className="text-slate-500 hover:text-white">✕</button>
          </div>
          {mobileMode ? (
            <div className="grid grid-cols-2 gap-1.5 font-mono">
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-sky-300">← / →</span><br />Steer</div>
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-emerald-300">GAS</span><br />Throttle</div>
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-rose-300">BRAKE</span><br />Brake</div>
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-amber-300">HB</span><br />Handbrake</div>
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-slate-200">CAM</span><br />Camera</div>
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-slate-200">RESET</span><br />Respawn</div>
            </div>
          ) : (
            <div className="grid grid-cols-2 gap-1.5 font-mono">
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-emerald-300">W / ↑</span><br />Throttle</div>
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-rose-300">S / ↓</span><br />Brake</div>
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-sky-300">A / D</span><br />Steer</div>
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-amber-300">SPACE</span><br />Handbrake</div>
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-slate-200">C / R</span><br />Camera / reset</div>
              <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-slate-200">H / T</span><br />HUD / telemetry</div>
            </div>
          )}
        </div>
      )}

      {/* Keyboard visualizer remains desktop-only even if the viewport is narrow. */}
      {toolbarExpanded && !mobileMode && (
        <div className="pointer-events-none absolute bottom-3 left-3 z-10 hidden items-end gap-1.5 sm:flex">
          <div className="grid grid-cols-3 gap-1 rounded-xl border border-slate-800/70 bg-slate-950/70 p-1.5 backdrop-blur-md">
            <div />
            <KeyCap active={isW} label="W" activeClass="bg-emerald-400 text-slate-950" />
            <div />
            <KeyCap active={isA} label="A" activeClass="bg-sky-400 text-slate-950" />
            <KeyCap active={isS} label="S" activeClass="bg-rose-400 text-slate-950" />
            <KeyCap active={isD} label="D" activeClass="bg-sky-400 text-slate-950" />
          </div>
          <div className={`rounded-xl border px-2 py-2 font-mono text-[9px] font-bold ${isSpace ? 'border-amber-300 bg-amber-400 text-slate-950' : 'border-slate-800/70 bg-slate-950/70 text-slate-500'}`}>
            SPACE
          </div>
        </div>
      )}

      {/* Touch-first driving layer. It is OFF by default on desktop and auto-enables on phones/tablets. */}
      {mobileMode && (
        <div
          id="mobile-driving-controls"
          className="pointer-events-none absolute inset-x-0 bottom-0 z-20 flex items-end justify-between gap-3"
          style={{
            paddingBottom: 'max(0.75rem, env(safe-area-inset-bottom))',
            paddingLeft: 'max(0.75rem, env(safe-area-inset-left))',
            paddingRight: 'max(0.75rem, env(safe-area-inset-right))',
          }}
        >
          <div className="pointer-events-auto flex gap-2 rounded-3xl border border-slate-700/55 bg-slate-950/36 p-2 shadow-2xl backdrop-blur-sm">
            <TouchButton
              label="←"
              ariaLabel="Steer left"
              tone="steering"
              onDown={() => onTouchInput('steerLeft', true)}
              onUp={() => onTouchInput('steerLeft', false)}
            />
            <TouchButton
              label="→"
              ariaLabel="Steer right"
              tone="steering"
              onDown={() => onTouchInput('steerRight', true)}
              onUp={() => onTouchInput('steerRight', false)}
            />
          </div>

          <div className="pointer-events-auto rounded-3xl border border-slate-700/55 bg-slate-950/36 p-2 shadow-2xl backdrop-blur-sm">
            <div className="mb-2 flex justify-end gap-2">
              <TapButton label="CAM" ariaLabel="Change camera" onClick={onNextCamera} />
              <TapButton label="RESET" ariaLabel="Reset car" onClick={onReset} />
              <TouchButton
                label="HB"
                ariaLabel="Handbrake"
                tone="handbrake"
                compact
                onDown={() => onTouchInput('handbrake', true)}
                onUp={() => onTouchInput('handbrake', false)}
              />
            </div>
            <div className="flex gap-2">
              <TouchButton
                label="BRAKE"
                ariaLabel="Brake"
                tone="brake"
                onDown={() => onTouchInput('brake', true)}
                onUp={() => onTouchInput('brake', false)}
              />
              <TouchButton
                label="GAS"
                ariaLabel="Throttle"
                tone="throttle"
                onDown={() => onTouchInput('throttle', true)}
                onUp={() => onTouchInput('throttle', false)}
              />
            </div>
          </div>
        </div>
      )}
    </>
  );
};

const KeyCap: React.FC<{ active: boolean; label: string; activeClass: string }> = ({ active, label, activeClass }) => (
  <div className={`flex h-7 w-7 items-center justify-center rounded-lg border font-mono text-[9px] font-bold ${active ? `${activeClass} border-white/30` : 'border-slate-800 bg-slate-900/90 text-slate-500'}`}>
    {label}
  </div>
);

const TOUCH_TONE_CLASSES = {
  steering: 'active:border-sky-200 active:bg-sky-400 active:text-slate-950',
  throttle: 'active:border-emerald-200 active:bg-emerald-400 active:text-slate-950',
  brake: 'active:border-rose-200 active:bg-rose-400 active:text-slate-950',
  handbrake: 'active:border-amber-200 active:bg-amber-400 active:text-slate-950',
} as const;

const pulseHaptic = () => {
  if (typeof navigator !== 'undefined' && 'vibrate' in navigator) {
    navigator.vibrate?.(8);
  }
};

const TouchButton: React.FC<{
  label: string;
  ariaLabel: string;
  onDown: () => void;
  onUp: () => void;
  tone: keyof typeof TOUCH_TONE_CLASSES;
  compact?: boolean;
}> = ({ label, ariaLabel, onDown, onUp, tone, compact = false }) => {
  const release = (event: React.PointerEvent<HTMLButtonElement>) => {
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    onUp();
  };

  return (
    <button
      type="button"
      aria-label={ariaLabel}
      onContextMenu={(event) => event.preventDefault()}
      onPointerDown={(event) => {
        event.preventDefault();
        event.currentTarget.setPointerCapture(event.pointerId);
        pulseHaptic();
        onDown();
      }}
      onPointerUp={release}
      onPointerCancel={release}
      onLostPointerCapture={onUp}
      className={`${compact ? 'h-11 min-w-14 px-2 text-[10px]' : 'h-[4.5rem] w-[4.5rem] text-sm'} ${TOUCH_TONE_CLASSES[tone]} flex touch-none select-none items-center justify-center rounded-2xl border border-slate-600/80 bg-slate-950/88 font-black tracking-wide text-slate-100 shadow-xl backdrop-blur-md transition-transform active:scale-95`}
    >
      {label}
    </button>
  );
};

const TapButton: React.FC<{
  label: string;
  ariaLabel: string;
  onClick: () => void;
}> = ({ label, ariaLabel, onClick }) => (
  <button
    type="button"
    aria-label={ariaLabel}
    onClick={() => {
      pulseHaptic();
      onClick();
    }}
    className="flex h-11 min-w-14 touch-manipulation select-none items-center justify-center rounded-2xl border border-slate-700/80 bg-slate-950/88 px-2 text-[9px] font-black tracking-wide text-slate-300 shadow-lg backdrop-blur-md active:scale-95 active:bg-slate-700 active:text-white"
  >
    {label}
  </button>
);
