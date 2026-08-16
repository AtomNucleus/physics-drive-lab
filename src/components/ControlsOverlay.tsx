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
  Eraser,
  HelpCircle,
  RotateCcw,
  Sliders,
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
  const activePreset = VEHICLE_PRESETS[activePresetKey] || VEHICLE_PRESETS.sportGT;

  const isW = activeKeys['KeyW'] || activeKeys['ArrowUp'];
  const isS = activeKeys['KeyS'] || activeKeys['ArrowDown'];
  const isA = activeKeys['KeyA'] || activeKeys['ArrowLeft'];
  const isD = activeKeys['KeyD'] || activeKeys['ArrowRight'];
  const isSpace = activeKeys['Space'];

  return (
    <>
      {/* Compact utility bar. Secondary controls are one click away instead of permanently occupying the view. */}
      <div className="absolute left-1/2 top-3 z-20 -translate-x-1/2">
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
          <div className="mt-1.5 flex items-center justify-center gap-1 rounded-2xl border border-slate-800/75 bg-slate-950/88 p-1 shadow-2xl backdrop-blur-xl">
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
              <Eraser size={14} />
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
          <div className="grid grid-cols-2 gap-1.5 font-mono">
            <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-emerald-300">W / ↑</span><br />Throttle</div>
            <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-rose-300">S / ↓</span><br />Brake</div>
            <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-sky-300">A / D</span><br />Steer</div>
            <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-amber-300">SPACE</span><br />Handbrake</div>
            <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-slate-200">C / R</span><br />Camera / reset</div>
            <div className="rounded-lg bg-slate-900/65 p-2"><span className="text-slate-200">H / T</span><br />HUD / telemetry</div>
          </div>
        </div>
      )}

      {/* Keyboard visualizer now follows the expanded utility state, so it is not permanent screen furniture. */}
      {toolbarExpanded && (
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

      {/* Mobile driving controls remain available regardless of HUD density. */}
      <div className="pointer-events-auto absolute bottom-3 right-3 z-20 flex items-end gap-2 sm:hidden">
        <div className="flex gap-1.5">
          <TouchButton label="←" onDown={() => onTouchInput('steerLeft', true)} onUp={() => onTouchInput('steerLeft', false)} />
          <TouchButton label="→" onDown={() => onTouchInput('steerRight', true)} onUp={() => onTouchInput('steerRight', false)} />
        </div>
        <div className="flex flex-col gap-1.5">
          <TouchButton label="GAS" compact activeClass="active:bg-emerald-400" onDown={() => onTouchInput('throttle', true)} onUp={() => onTouchInput('throttle', false)} />
          <TouchButton label="BRK" compact activeClass="active:bg-rose-400" onDown={() => onTouchInput('brake', true)} onUp={() => onTouchInput('brake', false)} />
        </div>
      </div>
    </>
  );
};

const KeyCap: React.FC<{ active: boolean; label: string; activeClass: string }> = ({ active, label, activeClass }) => (
  <div className={`flex h-7 w-7 items-center justify-center rounded-lg border font-mono text-[9px] font-bold ${active ? `${activeClass} border-white/30` : 'border-slate-800 bg-slate-900/90 text-slate-500'}`}>
    {label}
  </div>
);

const TouchButton: React.FC<{
  label: string;
  onDown: () => void;
  onUp: () => void;
  compact?: boolean;
  activeClass?: string;
}> = ({ label, onDown, onUp, compact = false, activeClass = 'active:bg-sky-400' }) => (
  <button
    onPointerDown={onDown}
    onPointerUp={onUp}
    onPointerCancel={onUp}
    onPointerLeave={onUp}
    className={`${compact ? 'h-10 w-12 text-[9px]' : 'h-12 w-12 text-lg'} ${activeClass} flex touch-none items-center justify-center rounded-xl border border-slate-700 bg-slate-950/88 font-bold text-slate-200 shadow-lg backdrop-blur-md`}
  >
    {label}
  </button>
);
