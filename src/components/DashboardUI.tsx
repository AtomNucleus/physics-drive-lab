import React from 'react';
import { VehicleState, VehicleConfig } from '../types';
import { Flame, Activity, Timer, Wind, Eye, Sparkles, ShieldAlert, Cpu } from 'lucide-react';

interface DashboardUIProps {
  state: VehicleState;
  config: VehicleConfig;
  useMph: boolean;
  onToggleUnit: () => void;
  onToggleAuto: () => void;
  showTelemetry: boolean;
  onToggleAbs?: () => void;
  onToggleTcs?: () => void;
  onToggleForceVectors?: () => void;
  onTriggerClutchKick?: () => void;
  onToggleDrs?: () => void;
}

export const DashboardUI: React.FC<DashboardUIProps> = ({
  state,
  config,
  useMph,
  onToggleUnit,
  onToggleAuto,
  showTelemetry,
  onToggleForceVectors,
  onTriggerClutchKick,
  onToggleDrs,
}) => {
  const displaySpeed = useMph ? Math.round(state.speedMph) : Math.round(state.speedKmh);
  const unitLabel = useMph ? 'MPH' : 'KM/H';

  const rpmPercent = Math.min(100, Math.max(0, (state.rpm / config.maxRpm) * 100));
  const isRedlining = state.rpm > config.revLimiterRpm * 0.96;

  // Gear label
  const gearText = state.gear === -1 ? 'R' : state.gear === 0 ? 'N' : `${state.gear}`;

  // Body roll and pitch in degrees
  const rollDeg = (state.roll * 180) / Math.PI;
  const pitchDeg = (state.pitch * 180) / Math.PI;

  const boostPercent = Math.min(100, Math.max(0, (state.turboBoostPsi / config.turboBoostMaxPsi) * 100));

  // Temperature color helper
  const getTempColor = (temp: number) => {
    if (temp < 60) return 'text-sky-400 bg-sky-950/60 border-sky-800';
    if (temp <= 95) return 'text-emerald-400 bg-emerald-950/60 border-emerald-800';
    if (temp <= 115) return 'text-amber-400 bg-amber-950/60 border-amber-800';
    return 'text-rose-400 bg-rose-950/60 border-rose-800 animate-pulse';
  };

  // Surface status label helper
  const getSurfaceLabel = (wheelSurface: string, friction: number) => {
    switch (wheelSurface) {
      case 'wet':
        return { label: 'WET SKIDPAD', color: 'text-cyan-400 border-cyan-500/40 bg-cyan-950/50' };
      case 'racing_line':
        return { label: 'RUBBERED GROOVE', color: 'text-emerald-400 border-emerald-500/40 bg-emerald-950/50' };
      case 'kerb':
        return { label: 'RUMBLE KERB', color: 'text-amber-400 border-amber-500/40 bg-amber-950/50' };
      case 'gravel':
        return { label: 'GRAVEL RUNOFF', color: 'text-rose-400 border-rose-500/40 bg-rose-950/50' };
      case 'marbles':
        return { label: 'OFF-LINE MARBLES', color: 'text-orange-400 border-orange-500/40 bg-orange-950/50' };
      default:
        return { label: 'CLEAN ASPHALT', color: 'text-slate-300 border-slate-700/60 bg-slate-900/60' };
    }
  };

  const primarySurface = state.wheels[0]?.surfaceType || 'asphalt';
  const primaryFriction = state.wheels[0]?.surfaceFriction || 1.0;
  const surfaceInfo = getSurfaceLabel(primarySurface, primaryFriction);

  return (
    <div id="driving-hud" className="pointer-events-none absolute inset-0 select-none overflow-hidden font-sans">
      {/* Top Left: Quick Telemetry, Surface & Status Badges */}
      <div className="absolute top-4 left-4 flex flex-col gap-2 pointer-events-auto">
        <div className="flex items-center gap-2 bg-slate-900/90 backdrop-blur-md border border-slate-700/70 rounded-2xl px-3.5 py-2 text-xs text-slate-200 shadow-2xl">
          <span className="font-bold text-sky-400 tracking-wider">PACEJKA '96</span>
          <span className="text-slate-600">|</span>
          <span className="text-slate-300 font-mono text-[11px]">
            {config.drivetrain} • {config.differentialType.replace(/_/g, ' ')}
          </span>
          <span className="inline-block w-2 h-2 rounded-full bg-emerald-400 animate-pulse"></span>
        </div>

        {/* Dynamic Surface Grip Reader Badge */}
        <div className={`flex items-center justify-between gap-2 border rounded-2xl px-3 py-1.5 text-[11px] font-mono shadow-lg backdrop-blur-md ${surfaceInfo.color}`}>
          <span className="font-bold uppercase tracking-wider">{surfaceInfo.label}</span>
          <span className="font-bold opacity-90">{primaryFriction.toFixed(2)}μ</span>
        </div>

        {/* Active Aero / DRS & Airbrake Status */}
        <div className="flex items-center gap-2">
          {config.drsEnabled && (
            <button
              id="drs-toggle-btn"
              onClick={onToggleDrs}
              className={`flex items-center gap-1.5 px-3 py-1.5 rounded-xl text-xs font-bold font-mono uppercase border transition-all cursor-pointer shadow-md ${
                state.drsActive
                  ? 'bg-emerald-600 text-white border-emerald-400 shadow-emerald-600/40 animate-pulse'
                  : 'bg-slate-900/85 text-slate-400 border-slate-700 hover:text-slate-200'
              }`}
              title="Toggle Drag Reduction System wing flap"
            >
              <Wind size={13} />
              <span>DRS {state.drsActive ? 'OPEN' : 'CLOSED'}</span>
            </button>
          )}

          {state.airbrakeActive && (
            <div className="flex items-center gap-1.5 bg-amber-500 text-slate-950 font-black text-xs uppercase px-3 py-1.5 rounded-xl shadow-lg shadow-amber-500/30 animate-pulse">
              <span>AIRBRAKE +38°</span>
            </div>
          )}

          {/* Clutch Kick Drift Trigger Button */}
          <button
            id="clutch-kick-btn"
            onClick={onTriggerClutchKick}
            className={`flex items-center gap-1 px-3 py-1.5 rounded-xl text-xs font-bold font-mono uppercase border transition-all cursor-pointer shadow-md ${
              state.clutchKickImpulse > 0.1
                ? 'bg-amber-500 text-slate-950 border-amber-300 shadow-amber-500/40 scale-105'
                : 'bg-slate-900/85 text-amber-400 border-amber-500/40 hover:bg-amber-950/40'
            }`}
            title="Trigger Instant Clutch Kick Drift Initiation"
          >
            <Sparkles size={13} />
            <span>CLUTCH KICK</span>
          </button>
        </div>

        {/* Diffuser Stall Alert */}
        {state.diffuserStalled && (
          <div className="flex items-center gap-2 bg-rose-600 text-slate-50 font-black text-xs uppercase px-3 py-1.5 rounded-xl shadow-lg shadow-rose-600/30 animate-pulse">
            <ShieldAlert size={14} />
            <span>DIFFUSER STALL (BOTTOMED OUT)</span>
          </div>
        )}

        {/* Rev Limiter Spark Cut Alert */}
        {state.isRevLimiting && (
          <div className="flex items-center gap-2 bg-rose-500 text-slate-950 font-black text-xs uppercase px-3 py-1.5 rounded-xl shadow-lg shadow-rose-500/30 animate-ping">
            <Cpu size={14} />
            <span>REV-LIMIT SPARK CUT</span>
          </div>
        )}

        {/* Drift Score Badge */}
        {state.isDrifting && (
          <div className="flex items-center gap-2.5 bg-amber-500 text-slate-950 font-black text-xs uppercase px-3.5 py-1.5 rounded-xl shadow-lg shadow-amber-500/25 animate-bounce">
            <Flame size={15} />
            <span>DRIFT {Math.abs(Math.round(state.driftAngleDeg))}°</span>
            <span className="bg-slate-950 text-amber-400 px-1.5 py-0.5 rounded text-[11px] font-mono">
              +{state.driftScore} PTS
            </span>
          </div>
        )}

        {/* Performance Sprint Trap Overlay */}
        <div className="bg-slate-900/85 backdrop-blur-md border border-slate-800 rounded-2xl p-3 text-xs text-slate-300 shadow-xl w-64">
          <div className="flex items-center justify-between font-bold text-[10px] text-slate-400 uppercase tracking-wider mb-2 border-b border-slate-800 pb-1">
            <span className="flex items-center gap-1.5 text-sky-400">
              <Timer size={13} />
              Sprint Telemetry
            </span>
            <span className="font-mono text-emerald-400">{state.performanceTimer.isTimingSprint ? 'TIMING...' : 'READY'}</span>
          </div>
          <div className="grid grid-cols-2 gap-2 text-[11px] font-mono">
            <div>
              <span className="text-slate-400 text-[10px] block">0-60 MPH (0-100)</span>
              <span className="font-bold text-slate-100">
                {state.performanceTimer.zeroToSixtyTime ? `${state.performanceTimer.zeroToSixtyTime}s` : '--'}
              </span>
            </div>
            <div>
              <span className="text-slate-400 text-[10px] block">1/4 Mile (402m)</span>
              <span className="font-bold text-amber-400">
                {state.performanceTimer.quarterMileTime ? `${state.performanceTimer.quarterMileTime}s` : '--'}
              </span>
            </div>
            <div className="col-span-2 flex justify-between items-center pt-1 border-t border-slate-800/60 text-[10px]">
              <span className="text-slate-400">Peak Lateral G</span>
              <span className="font-bold text-sky-400">{state.performanceTimer.peakLateralG} G</span>
            </div>
          </div>
        </div>
      </div>

      {/* Top Right: Advanced 4-Tire, 2-Way Damping & G-G Diagram Window */}
      {showTelemetry && (
        <div className="absolute top-4 right-4 pointer-events-auto flex flex-col gap-2.5">
          <div className="bg-slate-900/90 backdrop-blur-xl border border-slate-700/80 rounded-3xl p-4 text-xs text-slate-300 w-88 shadow-2xl">
            <div className="flex justify-between items-center mb-3 pb-2 border-b border-slate-800">
              <span className="font-bold text-slate-100 tracking-wider text-[11px] uppercase flex items-center gap-1.5">
                <Activity size={14} className="text-sky-400" />
                Kinematics & Dynamics
              </span>
              <span className="font-mono text-emerald-400 text-[10px]">
                {config.mass} kg • {Math.round(state.aeroDownforceTotalN)}N AERO
              </span>
            </div>

            {/* 3D Force Vectors Quick Toggle */}
            <div className="flex items-center justify-between pb-2 mb-2 border-b border-slate-800/80">
              <span className="text-[10px] text-slate-400 uppercase font-semibold flex items-center gap-1">
                <Eye size={12} className="text-sky-400" />
                3D Force Vectors & Puck
              </span>
              <button
                id="toggle-3d-vectors-btn"
                onClick={onToggleForceVectors}
                className={`text-[10px] font-mono px-2 py-0.5 rounded border transition-colors cursor-pointer ${
                  state.showForceVectors3D
                    ? 'bg-sky-950 text-sky-300 border-sky-600'
                    : 'bg-slate-950 text-slate-500 border-slate-800'
                }`}
              >
                {state.showForceVectors3D ? 'VISIBLE' : 'HIDDEN'}
              </button>
            </div>

            {/* Steering Rack Self-Aligning Torque */}
            <div className="flex justify-between items-center mb-2.5 text-[11px] bg-slate-950/60 p-2 rounded-xl border border-slate-800/80">
              <span className="text-slate-400">Self-Aligning Mz</span>
              <span className="font-mono font-bold text-sky-300">
                {state.steeringRackTorque.toFixed(1)} Nm (Trail: {(state.wheels[0].pneumaticTrail * 1000).toFixed(0)}mm)
              </span>
            </div>

            {/* Diffuser Clearance */}
            {config.groundEffectUnderbody && (
              <div className="flex justify-between items-center mb-2.5 text-[11px] bg-slate-950/60 p-2 rounded-xl border border-slate-800/80">
                <span className="text-slate-400">Underbody Clearance</span>
                <span className={`font-mono font-bold ${state.diffuserStalled ? 'text-rose-400' : 'text-emerald-400'}`}>
                  {(state.diffuserRideHeightM * 100).toFixed(1)} cm {state.diffuserStalled ? '(STALL)' : '(VENTURI)'}
                </span>
              </div>
            )}

            {/* Body Roll Bar */}
            <div className="space-y-1 mb-2.5">
              <div className="flex justify-between text-[11px]">
                <span className="text-slate-400">Chassis Body Roll</span>
                <span className={`font-mono font-bold ${Math.abs(rollDeg) > 4 ? 'text-amber-400' : 'text-slate-200'}`}>
                  {rollDeg >= 0 ? '+' : ''}{rollDeg.toFixed(1)}° {rollDeg > 0.5 ? '(Right)' : rollDeg < -0.5 ? '(Left)' : ''}
                </span>
              </div>
              <div className="relative h-2 bg-slate-950 rounded-full overflow-hidden border border-slate-800">
                <div className="absolute top-0 bottom-0 left-1/2 w-0.5 bg-slate-600 z-10"></div>
                <div
                  className={`h-full transition-all duration-75 ${rollDeg >= 0 ? 'bg-indigo-500' : 'bg-sky-500'}`}
                  style={{
                    marginLeft: rollDeg < 0 ? `${50 - Math.min(50, Math.abs(rollDeg) * 4)}%` : '50%',
                    width: `${Math.min(50, Math.abs(rollDeg) * 4)}%`,
                  }}
                ></div>
              </div>
            </div>

            {/* Pitch Dive / Squat Bar */}
            <div className="space-y-1 mb-3">
              <div className="flex justify-between text-[11px]">
                <span className="text-slate-400">Pitch (Dive / Squat)</span>
                <span className={`font-mono font-bold ${Math.abs(pitchDeg) > 3 ? 'text-amber-400' : 'text-slate-200'}`}>
                  {pitchDeg >= 0 ? '+' : ''}{pitchDeg.toFixed(1)}° {pitchDeg > 0.4 ? '(Dive)' : pitchDeg < -0.4 ? '(Squat)' : ''}
                </span>
              </div>
              <div className="relative h-2 bg-slate-950 rounded-full overflow-hidden border border-slate-800">
                <div className="absolute top-0 bottom-0 left-1/2 w-0.5 bg-slate-600 z-10"></div>
                <div
                  className={`h-full transition-all duration-75 ${pitchDeg >= 0 ? 'bg-rose-500' : 'bg-amber-500'}`}
                  style={{
                    marginLeft: pitchDeg < 0 ? `${50 - Math.min(50, Math.abs(pitchDeg) * 5)}%` : '50%',
                    width: `${Math.min(50, Math.abs(pitchDeg) * 5)}%`,
                  }}
                ></div>
              </div>
            </div>

            {/* 4-Wheel Thermodynamic, Damper Velocity & Wear Map */}
            <div className="pt-2 border-t border-slate-800">
              <div className="text-[10px] text-slate-400 uppercase tracking-wider mb-2 font-semibold flex justify-between items-center">
                <span>4-Corner Dynamics & Wear</span>
                <span className="text-[9px] text-emerald-400 font-mono">Opt 85°C</span>
              </div>
              <div className="grid grid-cols-2 gap-2 text-[10px]">
                {state.wheels.map((w) => {
                  const compPercent = Math.round(w.suspensionCompression * 100);
                  const loadKg = Math.round(w.suspensionForce / 9.81);
                  const tempBadgeClass = getTempColor(w.temperature);

                  return (
                    <div
                      key={w.id}
                      className={`p-2 rounded-xl border transition-colors ${
                        w.bumpStopEngaged
                          ? 'bg-rose-950/60 border-rose-500 text-rose-200'
                          : w.isSkidding
                          ? 'bg-amber-950/40 border-amber-500/80 text-amber-200'
                          : 'bg-slate-950/60 border-slate-800 text-slate-300'
                      }`}
                    >
                      <div className="flex justify-between font-bold text-[10px] mb-1">
                        <span>{w.id} {w.isFront ? 'FRONT' : 'REAR'}</span>
                        <span className="font-mono text-emerald-400">{loadKg}kg</span>
                      </div>
                      <div className="h-1.5 bg-slate-900 rounded-full overflow-hidden mb-1.5">
                        <div
                          className={`h-full ${w.bumpStopEngaged ? 'bg-rose-500' : w.isSkidding ? 'bg-amber-500' : 'bg-sky-400'}`}
                          style={{ width: `${compPercent}%` }}
                        ></div>
                      </div>
                      <div className="flex justify-between text-[9px] font-mono mb-1">
                        <span className={`px-1 py-0.5 rounded border ${tempBadgeClass}`}>
                          {Math.round(w.temperature)}°C
                        </span>
                        <span className="text-slate-400">{w.pressurePsi.toFixed(1)} PSI</span>
                      </div>
                      <div className="flex justify-between text-[9px] text-slate-400 font-mono">
                        <span>Wear {w.tireWearPercent.toFixed(0)}%</span>
                        <span className={w.bumpStopEngaged ? 'text-rose-400 font-bold' : w.isSkidding ? 'text-amber-400 font-bold' : ''}>
                          {w.bumpStopEngaged ? 'BUMP-STOP' : w.isSkidding ? 'SKID' : 'GRIP'}
                        </span>
                      </div>
                      {/* Dynamic Damper Velocity & Slip Angle */}
                      <div className="mt-1 pt-1 border-t border-slate-800/80 flex justify-between text-[8px] font-mono text-slate-400">
                        <span>{(w.slipAngle * (180 / Math.PI)).toFixed(1)}° slip</span>
                        <span>{w.damperVelocity >= 0 ? '+' : ''}{w.damperVelocity.toFixed(2)} m/s</span>
                      </div>
                    </div>
                  );
                })}
              </div>
            </div>

            {/* G-G Friction Diagram with Multi-Point History Trace */}
            <div className="mt-3 pt-2 border-t border-slate-800 flex items-center justify-between">
              <div className="space-y-0.5 text-[10px]">
                <div className="text-slate-400 font-semibold uppercase">Lateral G</div>
                <div className="font-mono text-sky-400 font-bold text-sm">
                  {Math.abs(state.lateralG).toFixed(2)} G{' '}
                  <span className="text-[10px] text-slate-400 font-normal">
                    {state.lateralG > 0.08 ? 'R' : state.lateralG < -0.08 ? 'L' : ''}
                  </span>
                </div>
                <div className="text-slate-400 font-semibold uppercase">Longitudinal G</div>
                <div className="font-mono text-amber-400 font-bold text-sm">
                  {state.longitudinalG >= 0 ? '+' : ''}{state.longitudinalG.toFixed(2)} G{' '}
                  <span className="text-[10px] text-slate-400 font-normal">
                    {state.longitudinalG > 0.08 ? 'ACC' : state.longitudinalG < -0.08 ? 'BRK' : ''}
                  </span>
                </div>
              </div>

              {/* 2D G-G Diagram with G-force trace scatter */}
              <div className="relative w-20 h-20 rounded-full border border-slate-700 bg-slate-950/95 flex items-center justify-center shadow-inner select-none">
                <div className="absolute w-full h-px bg-slate-800"></div>
                <div className="absolute h-full w-px bg-slate-800"></div>
                <div className="w-11 h-11 rounded-full border border-slate-800/90"></div>
                <div className="absolute inset-1 rounded-full border border-sky-500/20"></div>

                <span className="absolute top-0.5 text-[8px] font-mono font-bold text-slate-400">BRK</span>
                <span className="absolute bottom-0.5 text-[8px] font-mono font-bold text-slate-400">ACC</span>
                <span className="absolute left-1 text-[8px] font-mono font-bold text-slate-400">L</span>
                <span className="absolute right-1 text-[8px] font-mono font-bold text-slate-400">R</span>

                {/* Trace history points */}
                {state.gForceHistory.map((gPoint, i) => {
                  const opacity = (i / Math.max(1, state.gForceHistory.length)) * 0.45;
                  return (
                    <div
                      key={i}
                      className="absolute w-1 h-1 rounded-full bg-cyan-400 pointer-events-none"
                      style={{
                        transform: `translate(${Math.max(-32, Math.min(32, gPoint.lat * 20))}px, ${Math.max(-32, Math.min(32, gPoint.long * 20))}px)`,
                        opacity,
                      }}
                    />
                  );
                })}

                {/* Live G-Dot */}
                <div
                  className="absolute w-2.5 h-2.5 rounded-full bg-cyan-400 shadow-md shadow-cyan-400/90 transition-all duration-75 z-10"
                  style={{
                    transform: `translate(${Math.max(-32, Math.min(32, state.lateralG * 20))}px, ${Math.max(-32, Math.min(32, state.longitudinalG * 20))}px)`,
                  }}
                ></div>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Bottom Center: Primary Gauge Cluster (Speedometer, F1 Shift Lights, RPM, Turbo Boost, Pedals) */}
      <div className="absolute bottom-6 left-1/2 -translate-x-1/2 pointer-events-auto flex items-end gap-5">
        <div className="relative bg-slate-950/90 backdrop-blur-2xl border border-slate-800/90 rounded-3xl p-5 shadow-2xl flex flex-col items-center gap-3">
          
          {/* F1 LED Shift Light Bar */}
          <div className="flex items-center gap-1.5 bg-slate-900/90 border border-slate-800 px-3 py-1 rounded-full shadow-inner">
            {/* Left 4 LEDs */}
            <span className={`w-3 h-2 rounded-sm transition-colors ${state.shiftLightStage >= 1 ? 'bg-emerald-500 shadow-emerald-500/50 shadow-sm' : 'bg-slate-800'}`} />
            <span className={`w-3 h-2 rounded-sm transition-colors ${state.shiftLightStage >= 1 ? 'bg-emerald-500 shadow-emerald-500/50 shadow-sm' : 'bg-slate-800'}`} />
            <span className={`w-3 h-2 rounded-sm transition-colors ${state.shiftLightStage >= 2 ? 'bg-amber-400 shadow-amber-400/50 shadow-sm' : 'bg-slate-800'}`} />
            <span className={`w-3 h-2 rounded-sm transition-colors ${state.shiftLightStage >= 3 ? 'bg-rose-500 shadow-rose-500/50 shadow-sm' : 'bg-slate-800'}`} />
            
            {/* Center Blue Shift Flasher */}
            <span className={`w-4 h-2 rounded-sm transition-all font-mono text-[7px] text-center font-bold flex items-center justify-center ${state.shiftLightStage >= 4 ? 'bg-cyan-400 text-slate-950 shadow-cyan-400 shadow-md animate-pulse' : 'bg-slate-800 text-transparent'}`}>
              !
            </span>

            {/* Right 4 LEDs */}
            <span className={`w-3 h-2 rounded-sm transition-colors ${state.shiftLightStage >= 3 ? 'bg-rose-500 shadow-rose-500/50 shadow-sm' : 'bg-slate-800'}`} />
            <span className={`w-3 h-2 rounded-sm transition-colors ${state.shiftLightStage >= 2 ? 'bg-amber-400 shadow-amber-400/50 shadow-sm' : 'bg-slate-800'}`} />
            <span className={`w-3 h-2 rounded-sm transition-colors ${state.shiftLightStage >= 1 ? 'bg-emerald-500 shadow-emerald-500/50 shadow-sm' : 'bg-slate-800'}`} />
            <span className={`w-3 h-2 rounded-sm transition-colors ${state.shiftLightStage >= 1 ? 'bg-emerald-500 shadow-emerald-500/50 shadow-sm' : 'bg-slate-800'}`} />
          </div>

          <div className="flex items-center gap-7">
            {/* Gear Indicator & Transmission */}
            <div className="flex flex-col items-center">
              <button
                id="toggle-auto-btn"
                onClick={onToggleAuto}
                className="text-[10px] uppercase font-bold tracking-widest text-slate-400 hover:text-sky-400 transition-colors px-2 py-0.5 rounded bg-slate-900 border border-slate-800 mb-2 cursor-pointer pointer-events-auto"
                title="Click to toggle Automatic/Manual transmission"
              >
                {state.isAutomatic ? 'AUTO' : 'MANUAL'}
              </button>
              <div className="relative flex items-center justify-center w-14 h-16 bg-gradient-to-b from-slate-900 to-slate-950 rounded-2xl border border-slate-800 shadow-inner">
                <span className={`text-4xl font-black font-mono ${state.gear === -1 ? 'text-amber-400' : 'text-slate-100'}`}>
                  {gearText}
                </span>
              </div>
            </div>

            {/* Speedometer, RPM Bar & Turbo Boost Bar */}
            <div className="flex flex-col items-center min-w-[200px]">
              {/* Digital Speed Readout */}
              <div className="flex items-baseline gap-2">
                <span className="text-6xl font-black font-mono tracking-tighter text-slate-50 drop-shadow-md">
                  {displaySpeed}
                </span>
                <button
                  id="toggle-unit-btn"
                  onClick={onToggleUnit}
                  className="text-xs font-bold text-slate-400 hover:text-sky-400 transition-colors cursor-pointer"
                  title="Toggle KM/H / MPH"
                >
                  {unitLabel}
                </button>
              </div>

              {/* RPM Gauge Bar */}
              <div className="w-full mt-2">
                <div className="flex justify-between items-center text-[10px] text-slate-400 font-mono mb-1">
                  <span>{Math.round(state.rpm)} RPM</span>
                  <span className={isRedlining ? 'text-rose-500 font-bold animate-pulse' : 'text-slate-500'}>
                    REDLINE {config.revLimiterRpm}
                  </span>
                </div>
                <div className="h-2.5 w-full bg-slate-900 rounded-full overflow-hidden border border-slate-800 flex">
                  <div
                    className={`h-full transition-all duration-75 ${
                      isRedlining
                        ? 'bg-gradient-to-r from-amber-500 via-rose-500 to-red-600 animate-pulse'
                        : 'bg-gradient-to-r from-sky-500 via-indigo-500 to-amber-400'
                    }`}
                    style={{ width: `${rpmPercent}%` }}
                  ></div>
                </div>
              </div>

              {/* Turbocharger Boost PSI Bar */}
              <div className="w-full mt-2">
                <div className="flex justify-between items-center text-[9px] font-mono text-slate-400 mb-0.5">
                  <span className="text-sky-400 font-bold flex items-center gap-1">
                    TURBO BOOST
                    {state.wastegateOpen && <span className="text-rose-400 text-[8px]">WASTEGATE</span>}
                  </span>
                  <span className="font-bold text-slate-200">{state.turboBoostPsi.toFixed(1)} PSI</span>
                </div>
                <div className="h-1.5 w-full bg-slate-900 rounded-full overflow-hidden border border-slate-800">
                  <div
                    className="h-full bg-gradient-to-r from-sky-500 to-cyan-300 transition-all duration-75"
                    style={{ width: `${boostPercent}%` }}
                  ></div>
                </div>
              </div>
            </div>

            {/* Pedal Inputs Visualizer (Gas / Brake / Clutch / Handbrake) */}
            <div className="flex items-end gap-2.5 border-l border-slate-800/80 pl-6">
              {/* Throttle (Gas) */}
              <div className="flex flex-col items-center gap-1">
                <div className="h-16 w-3.5 bg-slate-900 rounded-full overflow-hidden border border-slate-800 flex flex-col justify-end p-0.5">
                  <div
                    className="w-full bg-emerald-500 rounded-full transition-all duration-75"
                    style={{ height: `${Math.round(state.throttle * 100)}%` }}
                  ></div>
                </div>
                <span className="text-[9px] font-bold text-slate-400">GAS</span>
              </div>

              {/* Foot Brake */}
              <div className="flex flex-col items-center gap-1">
                <div className="h-16 w-3.5 bg-slate-900 rounded-full overflow-hidden border border-slate-800 flex flex-col justify-end p-0.5">
                  <div
                    className={`w-full rounded-full transition-all duration-75 ${
                      state.absActive ? 'bg-amber-400 animate-pulse' : 'bg-rose-500'
                    }`}
                    style={{ height: `${Math.round(state.brake * 100)}%` }}
                  ></div>
                </div>
                <span className="text-[9px] font-bold text-slate-400">{state.absActive ? 'ABS' : 'BRK'}</span>
              </div>

              {/* Clutch Visualizer */}
              <div className="flex flex-col items-center gap-1">
                <div className="h-16 w-3.5 bg-slate-900 rounded-full overflow-hidden border border-slate-800 flex flex-col justify-end p-0.5">
                  <div
                    className={`w-full rounded-full transition-all duration-75 ${
                      state.clutchKickImpulse > 0.1 ? 'bg-amber-400 animate-pulse' : 'bg-cyan-500'
                    }`}
                    style={{ height: `${Math.round(state.clutchPedal * 100)}%` }}
                  ></div>
                </div>
                <span className="text-[9px] font-bold text-slate-400">CLUTCH</span>
              </div>

              {/* E-Brake / Handbrake Indicator */}
              <div className="flex flex-col items-center gap-1">
                <div
                  className={`h-16 w-5 rounded-xl border flex items-center justify-center text-[10px] font-black transition-colors ${
                    state.handbrake
                      ? 'bg-amber-500 text-slate-950 border-amber-400 shadow-lg shadow-amber-500/30'
                      : 'bg-slate-900 text-slate-600 border-slate-800'
                  }`}
                >
                  (P)
                </div>
                <span className="text-[9px] font-bold text-slate-400">HAND</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};
