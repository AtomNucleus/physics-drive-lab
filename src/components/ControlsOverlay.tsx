import React from 'react';
import { CameraMode } from '../types';
import { VEHICLE_PRESETS } from '../physics/vehiclePresets';
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

const detectMobileDrivingMode = () => {
  if (typeof window === 'undefined' || typeof navigator === 'undefined') return false;

  const override = new URLSearchParams(window.location.search).get('mobileControls');
  if (override === '1' || override === 'true') return true;
  if (override === '0' || override === 'false') return false;

  const nav = navigator as Navigator & { userAgentData?: { mobile?: boolean } };
  const userAgent = navigator.userAgent || '';
  const platform = navigator.platform || '';
  const uaMobile = nav.userAgentData?.mobile === true || /Android|iPhone|iPod|IEMobile|Opera Mini/i.test(userAgent);
  const iPadLike = /iPad/i.test(userAgent) || (platform === 'MacIntel' && navigator.maxTouchPoints > 1);
  const coarsePointer = window.matchMedia('(pointer: coarse)').matches;
  const touchCapable = navigator.maxTouchPoints > 0 || 'ontouchstart' in window;
  const touchSizedViewport = window.innerWidth <= 1180;

  return uaMobile || iPadLike || (coarsePointer && touchCapable && touchSizedViewport);
};

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
  const [mobileMode, setMobileMode] = React.useState(false);
  const activePreset = VEHICLE_PRESETS[activePresetKey] || VEHICLE_PRESETS.sportGT;

  const isW = activeKeys['KeyW'] || activeKeys['ArrowUp'];
  const isS = activeKeys['KeyS'] || activeKeys['ArrowDown'];
  const isA = activeKeys['KeyA'] || activeKeys['ArrowLeft'];
  const isD = activeKeys['KeyD'] || activeKeys['ArrowRight'];
  const isSpace = activeKeys['Space'];

  React.useEffect(() => {
    const pointerQuery = window.matchMedia('(pointer: coarse)');
    const evaluate = () => setMobileMode(detectMobileDrivingMode());

    evaluate();
    pointerQuery.addEventListener?.('change', evaluate);
    window.addEventListener('resize', evaluate);
    window.addEventListener('orientationchange', evaluate);

    return () => {
      pointerQuery.removeEventListener?.('change', evaluate);
      window.removeEventListener('resize', evaluate);
      window.removeEventListener('orientationchange', evaluate);
    };
  }, []);

  React.useEffect(() => {
    if (mobileMode) {
      document.documentElement.dataset.drivingMobile = 'true';
      return () => {
        delete document.documentElement.dataset.drivingMobile;
      };
    }

    delete document.documentElement.dataset.drivingMobile;
    return undefined;
  }, [mobileMode]);

  return (
    <>
      {/* Compact utility bar. Secondary controls are one click away instead of permanently occupying the view. */}
      <div
        className="absolute left-1/2 top-3 z-20 -translate-x-1/2"
        style={mobileMode ? { top: 'max(0.75rem, env(safe-area-inset-top))' } : undefined}
      >
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
            <span className={mobileMode ? 'capitalize' : 'hidden capitalize sm:inline'}>{cameraMode}</span>
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
          <div className="mt-1.5 flex items-center justify-center gap-1 rounded-2xl border border-slate-800/75 bg-slate-950/88 p-1 shadow-2xl backdrop-blur-xl">
            <button
              id="tuning-btn"
              onClick={onOpenTuning}
              className="flex h-8 items-center gap-1 rounded-xl px-2 text-[10px] font-semibold text-slate-400 hover:bg-slate-800 hover:text-sky-300"
              title="Physics tuning (P)"
            >
              <Sliders size={14} />
              <span className={mobileMode ? 'hidden' : 'hidden sm:inline'}>Physics</span>
            </button>

            {onOpenTestRunner && (
              <button
                id="physics-tests-btn"
                onClick={onOpenTestRunner}
                className="flex h-8 items-center gap-1 rounded-xl px-2 text-[10px] font-semibold text-sky-300 hover:bg-sky-950/60"
                title="Physics tests (Y)"
              >
                <Cpu size={14} />
                <span className={mobileMode ? 'hidden' : 'hidden sm:inline'}>Tests</span>
              </button>
            )}

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
              Driving controls
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

      {/* Keyboard visualizer follows the expanded utility state and never appears in mobile mode. */}
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

      {mobileMode && (
        <MobileDrivingControls
          onTouchInput={onTouchInput}
          onNextCamera={onNextCamera}
          onReset={onReset}
        />
      )}
    </>
  );
};

const KeyCap: React.FC<{ active: boolean; label: string; activeClass: string }> = ({ active, label, activeClass }) => (
  <div className={`flex h-7 w-7 items-center justify-center rounded-lg border font-mono text-[9px] font-bold ${active ? `${activeClass} border-white/30` : 'border-slate-800 bg-slate-900/90 text-slate-500'}`}>
    {label}
  </div>
);

const MobileDrivingControls: React.FC<{
  onTouchInput: (action: TouchAction, active: boolean) => void;
  onNextCamera: () => void;
  onReset: () => void;
}> = ({ onTouchInput, onNextCamera, onReset }) => (
  <>
    <div id="mobile-landscape-hint" className="pointer-events-none absolute left-1/2 top-16 z-30 hidden -translate-x-1/2 rounded-full border border-slate-700/80 bg-slate-950/82 px-3 py-1.5 text-[9px] font-bold uppercase tracking-wider text-slate-300 shadow-xl backdrop-blur-lg">
      Rotate for the best driving view
    </div>

    <div
      id="mobile-driving-controls"
      className="pointer-events-none absolute inset-x-0 bottom-0 z-40 flex items-end justify-between gap-3"
      style={{
        paddingLeft: 'max(0.75rem, env(safe-area-inset-left))',
        paddingRight: 'max(0.75rem, env(safe-area-inset-right))',
        paddingBottom: 'max(0.75rem, env(safe-area-inset-bottom))',
      }}
    >
      <div className="pointer-events-auto flex flex-col items-start gap-1.5">
        <span className="pl-1 text-[8px] font-black uppercase tracking-[0.2em] text-slate-400/90">Steer</span>
        <div className="flex gap-2">
          <MobileTouchButton
            label="←"
            ariaLabel="Steer left"
            className="h-[4.75rem] w-[4.75rem] text-3xl active:border-sky-300 active:bg-sky-400 active:text-slate-950"
            onActiveChange={(active) => onTouchInput('steerLeft', active)}
          />
          <MobileTouchButton
            label="→"
            ariaLabel="Steer right"
            className="h-[4.75rem] w-[4.75rem] text-3xl active:border-sky-300 active:bg-sky-400 active:text-slate-950"
            onActiveChange={(active) => onTouchInput('steerRight', active)}
          />
        </div>
      </div>

      <div className="pointer-events-auto absolute bottom-0 left-1/2 flex -translate-x-1/2 gap-1.5" style={{ marginBottom: 'max(0.75rem, env(safe-area-inset-bottom))' }}>
        <button
          type="button"
          onClick={onNextCamera}
          className="flex h-9 items-center gap-1 rounded-full border border-slate-700 bg-slate-950/82 px-3 text-[9px] font-bold text-slate-200 shadow-lg backdrop-blur-lg active:bg-slate-800"
          aria-label="Change camera"
        >
          <Camera size={13} /> CAM
        </button>
        <button
          type="button"
          onClick={onReset}
          className="flex h-9 items-center gap-1 rounded-full border border-slate-700 bg-slate-950/82 px-3 text-[9px] font-bold text-slate-200 shadow-lg backdrop-blur-lg active:bg-amber-300 active:text-slate-950"
          aria-label="Reset car"
        >
          <RotateCcw size={13} /> RESET
        </button>
      </div>

      <div className="pointer-events-auto flex items-end gap-2">
        <MobileTouchButton
          label="HB"
          ariaLabel="Handbrake"
          className="h-14 w-14 text-[11px] active:border-amber-300 active:bg-amber-300 active:text-slate-950"
          onActiveChange={(active) => onTouchInput('handbrake', active)}
        />
        <MobileTouchButton
          label="BRAKE"
          ariaLabel="Brake"
          className="h-[5.75rem] w-[4.5rem] text-[10px] active:border-rose-300 active:bg-rose-400 active:text-slate-950"
          onActiveChange={(active) => onTouchInput('brake', active)}
        />
        <MobileTouchButton
          label="GAS"
          ariaLabel="Throttle"
          className="h-[6.75rem] w-[4.75rem] text-xs active:border-emerald-300 active:bg-emerald-400 active:text-slate-950"
          onActiveChange={(active) => onTouchInput('throttle', active)}
        />
      </div>
    </div>
  </>
);

const MobileTouchButton: React.FC<{
  label: string;
  ariaLabel: string;
  className: string;
  onActiveChange: (active: boolean) => void;
}> = ({ label, ariaLabel, className, onActiveChange }) => {
  const pointerIdRef = React.useRef<number | null>(null);

  const deactivate = (event?: React.PointerEvent<HTMLButtonElement>) => {
    if (event && pointerIdRef.current !== null && event.pointerId !== pointerIdRef.current) return;
    pointerIdRef.current = null;
    onActiveChange(false);
  };

  return (
    <button
      type="button"
      aria-label={ariaLabel}
      onPointerDown={(event) => {
        event.preventDefault();
        if (pointerIdRef.current !== null) return;
        pointerIdRef.current = event.pointerId;
        event.currentTarget.setPointerCapture?.(event.pointerId);
        onActiveChange(true);
      }}
      onPointerUp={deactivate}
      onPointerCancel={deactivate}
      onLostPointerCapture={deactivate}
      onContextMenu={(event) => event.preventDefault()}
      className={`${className} flex touch-none select-none items-center justify-center rounded-[1.35rem] border border-slate-500/70 bg-slate-950/68 font-black text-slate-100 shadow-2xl backdrop-blur-md transition-[transform,background-color,border-color] duration-75 active:scale-[0.97]`}
    >
      {label}
    </button>
  );
};
