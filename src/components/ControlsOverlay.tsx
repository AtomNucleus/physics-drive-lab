import React from 'react';
import { CameraMode, VehiclePreset } from '../types';
import { VEHICLE_PRESETS } from '../physics/vehiclePresets';
import {
  Camera,
  RotateCcw,
  Volume2,
  VolumeX,
  Sliders,
  Activity,
  Car,
  ChevronRight,
  Sparkles,
  HelpCircle,
  Cpu,
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
  // Touch inputs for mobile / click
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
  onSelectPreset,
  activeKeys,
  onTouchInput,
}) => {
  const [showHelp, setShowHelp] = React.useState(false);

  const activePreset = VEHICLE_PRESETS[activePresetKey] || VEHICLE_PRESETS.sportGT;

  const isW = activeKeys['KeyW'] || activeKeys['ArrowUp'];
  const isS = activeKeys['KeyS'] || activeKeys['ArrowDown'];
  const isA = activeKeys['KeyA'] || activeKeys['ArrowLeft'];
  const isD = activeKeys['KeyD'] || activeKeys['ArrowRight'];
  const isSpace = activeKeys['Space'];

  return (
    <>
      {/* Top Floating Control Bar */}
      <div className="absolute top-4 left-1/2 -translate-x-1/2 z-20 flex items-center gap-2 bg-slate-950/85 backdrop-blur-xl border border-slate-800/80 rounded-2xl p-1.5 shadow-2xl">
        {/* Preset Selector Dropdown / Quick Cycler */}
        <div className="relative group">
          <button
            id="preset-selector-btn"
            onClick={onOpenTuning}
            className="flex items-center gap-2 px-3 py-1.5 rounded-xl bg-slate-900/90 hover:bg-slate-800 text-xs font-bold text-slate-200 border border-slate-800 transition-colors cursor-pointer"
          >
            <span className="w-2.5 h-2.5 rounded-full" style={{ backgroundColor: activePreset.color }}></span>
            <span>{activePreset.name.split(' ')[0]} {activePreset.name.split(' ')[1]}</span>
            <ChevronRight size={14} className="text-slate-500" />
          </button>
        </div>

        {/* Camera Toggle */}
        <button
          id="camera-mode-btn"
          onClick={onNextCamera}
          className="flex items-center gap-1.5 px-3 py-1.5 rounded-xl bg-slate-900/80 hover:bg-slate-800 text-xs font-semibold text-slate-300 hover:text-sky-400 border border-slate-800/80 transition-colors cursor-pointer"
          title="Switch Camera (Press C)"
        >
          <Camera size={15} />
          <span className="capitalize">{cameraMode}</span>
        </button>

        {/* Physics Tuning Workshop */}
        <button
          id="tuning-btn"
          onClick={onOpenTuning}
          className="flex items-center gap-1.5 px-3 py-1.5 rounded-xl bg-slate-900/80 hover:bg-slate-800 text-xs font-semibold text-slate-300 hover:text-sky-400 border border-slate-800/80 transition-colors cursor-pointer"
          title="Tuning Workshop (Press P)"
        >
          <Sliders size={15} />
          <span>Physics</span>
        </button>

        {/* Physics 2.0 Headless Acceptance Tests */}
        {onOpenTestRunner && (
          <button
            id="physics-tests-btn"
            onClick={onOpenTestRunner}
            className="flex items-center gap-1.5 px-3 py-1.5 rounded-xl bg-sky-950/40 hover:bg-sky-900/60 text-xs font-semibold text-sky-300 border border-sky-600/40 transition-colors cursor-pointer"
            title="Headless Verification Suite (Press Y)"
          >
            <Cpu size={15} />
            <span>Tests</span>
          </button>
        )}

        {/* Telemetry Toggle */}
        <button
          id="telemetry-toggle-btn"
          onClick={onToggleTelemetry}
          className={`p-2 rounded-xl border transition-colors cursor-pointer ${
            showTelemetry
              ? 'bg-sky-500/20 border-sky-500/40 text-sky-400'
              : 'bg-slate-900/80 border-slate-800/80 text-slate-400 hover:text-slate-200'
          }`}
          title="Toggle Telemetry (Press T)"
        >
          <Activity size={16} />
        </button>

        {/* Sound Toggle */}
        <button
          id="sound-toggle-btn"
          onClick={onToggleMute}
          className={`p-2 rounded-xl border transition-colors cursor-pointer ${
            !isMuted
              ? 'bg-emerald-500/20 border-emerald-500/40 text-emerald-400'
              : 'bg-slate-900/80 border-slate-800/80 text-slate-400 hover:text-slate-200'
          }`}
          title="Toggle Engine Audio (Press U)"
        >
          {isMuted ? <VolumeX size={16} /> : <Volume2 size={16} />}
        </button>

        {/* Reset Car */}
        <button
          id="reset-car-btn"
          onClick={onReset}
          className="p-2 rounded-xl bg-slate-900/80 hover:bg-slate-800 text-slate-300 hover:text-amber-400 border border-slate-800/80 transition-colors cursor-pointer"
          title="Reset Position (Press R)"
        >
          <RotateCcw size={16} />
        </button>

        {/* Help Toggle */}
        <button
          id="help-toggle-btn"
          onClick={() => setShowHelp(!showHelp)}
          className="p-2 rounded-xl bg-slate-900/80 hover:bg-slate-800 text-slate-400 hover:text-slate-200 border border-slate-800/80 transition-colors cursor-pointer"
          title="Help & Driving Controls"
        >
          <HelpCircle size={16} />
        </button>
      </div>

      {/* Help & Key Guide Drawer */}
      {showHelp && (
        <div className="absolute top-20 left-1/2 -translate-x-1/2 z-30 w-full max-w-md bg-slate-900/95 backdrop-blur-xl border border-slate-700/80 rounded-3xl p-5 shadow-2xl text-xs text-slate-200 animate-fade-in">
          <div className="flex justify-between items-center mb-3 pb-2 border-b border-slate-800">
            <span className="font-bold text-sm text-slate-100 flex items-center gap-2">
              <Car size={16} className="text-sky-400" />
              Keyboard Driving Guide
            </span>
            <button
              onClick={() => setShowHelp(false)}
              className="text-slate-400 hover:text-slate-100 cursor-pointer"
            >
              ✕
            </button>
          </div>

          <div className="grid grid-cols-2 gap-2.5 mb-4">
            <div className="bg-slate-950/60 p-2.5 rounded-xl border border-slate-800/80">
              <div className="text-[10px] text-slate-400 uppercase font-semibold mb-1">Throttle / Accelerate</div>
              <div className="font-mono font-bold text-emerald-400">W / Up Arrow</div>
            </div>
            <div className="bg-slate-950/60 p-2.5 rounded-xl border border-slate-800/80">
              <div className="text-[10px] text-slate-400 uppercase font-semibold mb-1">Brake / Reverse</div>
              <div className="font-mono font-bold text-rose-400">S / Down Arrow</div>
            </div>
            <div className="bg-slate-950/60 p-2.5 rounded-xl border border-slate-800/80">
              <div className="text-[10px] text-slate-400 uppercase font-semibold mb-1">Steer Left / Right</div>
              <div className="font-mono font-bold text-sky-400">A / D or Left / Right</div>
            </div>
            <div className="bg-slate-950/60 p-2.5 rounded-xl border border-slate-800/80">
              <div className="text-[10px] text-slate-400 uppercase font-semibold mb-1">Handbrake (Drift!)</div>
              <div className="font-mono font-bold text-amber-400">Spacebar</div>
            </div>
            <div className="bg-slate-950/60 p-2.5 rounded-xl border border-slate-800/80">
              <div className="text-[10px] text-slate-400 uppercase font-semibold mb-1">Camera Mode</div>
              <div className="font-mono font-bold text-slate-300">C (Cycle Views)</div>
            </div>
            <div className="bg-slate-950/60 p-2.5 rounded-xl border border-slate-800/80">
              <div className="text-[10px] text-slate-400 uppercase font-semibold mb-1">Reset Car / Cones</div>
              <div className="font-mono font-bold text-slate-300">R</div>
            </div>
          </div>

          <div className="text-[11px] text-slate-400 leading-relaxed bg-slate-950/40 p-3 rounded-xl border border-slate-800/60">
            <span className="font-bold text-sky-400">Physics Pro Tip:</span> Tap the brake (S) on turn-in to transfer weight forward to the front tires for razor-sharp turn-in grip, or yank the Spacebar handbrake while steering to initiate deep power slides with realistic body roll!
          </div>
        </div>
      )}

      {/* Bottom Left: Live Keyboard Indicator */}
      <div className="absolute bottom-6 left-6 z-10 hidden sm:flex items-center gap-1.5 pointer-events-none bg-slate-950/70 backdrop-blur-md border border-slate-800/70 p-2.5 rounded-2xl">
        <div className="flex flex-col items-center gap-1">
          <div
            className={`w-9 h-9 rounded-xl border flex items-center justify-center font-mono font-bold text-xs transition-all ${
              isW
                ? 'bg-emerald-500 text-slate-950 border-emerald-400 shadow-md shadow-emerald-500/30 scale-95'
                : 'bg-slate-900/90 text-slate-300 border-slate-800'
            }`}
          >
            W
          </div>
          <div className="flex gap-1">
            <div
              className={`w-9 h-9 rounded-xl border flex items-center justify-center font-mono font-bold text-xs transition-all ${
                isA
                  ? 'bg-sky-500 text-slate-950 border-sky-400 shadow-md shadow-sky-500/30 scale-95'
                  : 'bg-slate-900/90 text-slate-300 border-slate-800'
              }`}
            >
              A
            </div>
            <div
              className={`w-9 h-9 rounded-xl border flex items-center justify-center font-mono font-bold text-xs transition-all ${
                isS
                  ? 'bg-rose-500 text-slate-950 border-rose-400 shadow-md shadow-rose-500/30 scale-95'
                  : 'bg-slate-900/90 text-slate-300 border-slate-800'
              }`}
            >
              S
            </div>
            <div
              className={`w-9 h-9 rounded-xl border flex items-center justify-center font-mono font-bold text-xs transition-all ${
                isD
                  ? 'bg-sky-500 text-slate-950 border-sky-400 shadow-md shadow-sky-500/30 scale-95'
                  : 'bg-slate-900/90 text-slate-300 border-slate-800'
              }`}
            >
              D
            </div>
          </div>
        </div>

        <div
          className={`h-[78px] px-3 rounded-xl border flex items-center justify-center font-mono font-bold text-xs transition-all ${
            isSpace
              ? 'bg-amber-500 text-slate-950 border-amber-400 shadow-md shadow-amber-500/30 scale-95'
              : 'bg-slate-900/90 text-slate-400 border-slate-800'
          }`}
        >
          SPACE (DRIFT)
        </div>
      </div>

      {/* Mobile / Touch Driving Pad (visible on small screens or touch) */}
      <div className="absolute bottom-6 right-6 z-20 flex sm:hidden items-center gap-3 pointer-events-auto">
        {/* Left / Right Steering */}
        <div className="flex gap-2">
          <button
            onPointerDown={() => onTouchInput('steerLeft', true)}
            onPointerUp={() => onTouchInput('steerLeft', false)}
            onPointerLeave={() => onTouchInput('steerLeft', false)}
            className="w-14 h-14 bg-slate-900/90 active:bg-sky-500 border border-slate-700 rounded-2xl flex items-center justify-center text-xl font-bold text-slate-200"
          >
            ←
          </button>
          <button
            onPointerDown={() => onTouchInput('steerRight', true)}
            onPointerUp={() => onTouchInput('steerRight', false)}
            onPointerLeave={() => onTouchInput('steerRight', false)}
            className="w-14 h-14 bg-slate-900/90 active:bg-sky-500 border border-slate-700 rounded-2xl flex items-center justify-center text-xl font-bold text-slate-200"
          >
            →
          </button>
        </div>

        {/* Gas & Brake */}
        <div className="flex flex-col gap-2">
          <button
            onPointerDown={() => onTouchInput('throttle', true)}
            onPointerUp={() => onTouchInput('throttle', false)}
            onPointerLeave={() => onTouchInput('throttle', false)}
            className="w-14 h-12 bg-emerald-600 active:bg-emerald-400 border border-emerald-400 rounded-2xl flex items-center justify-center text-xs font-bold text-slate-950"
          >
            GAS
          </button>
          <button
            onPointerDown={() => onTouchInput('brake', true)}
            onPointerUp={() => onTouchInput('brake', false)}
            onPointerLeave={() => onTouchInput('brake', false)}
            className="w-14 h-12 bg-rose-600 active:bg-rose-400 border border-rose-400 rounded-2xl flex items-center justify-center text-xs font-bold text-slate-950"
          >
            BRAKE
          </button>
        </div>
      </div>
    </>
  );
};
