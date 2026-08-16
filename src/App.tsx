import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { VehicleConfig, VehicleState, CameraMode } from './types';
import { DEFAULT_VEHICLE_CONFIG, VEHICLE_PRESETS } from './physics/vehiclePresets';
import { BMW_M5_2025_OVERRIDES } from './physics/m5G90';
import { VehiclePhysicsEngine } from './physics/vehiclePhysics';
import { CarRenderer } from './graphics/carRenderer';
import { EnvironmentManager } from './graphics/environment';
import { CameraController } from './graphics/cameraController';
import { globalAudio } from './audio/engineAudio';
import { DashboardUI } from './components/DashboardUI';
import { ControlsOverlay } from './components/ControlsOverlay';
import { TuningModal } from './components/TuningModal';
import { PhysicsTestRunnerModal } from './components/PhysicsTestRunnerModal';

const INITIAL_PRESET_KEY = 'm5G90';
const INITIAL_CONFIG: VehicleConfig = { ...DEFAULT_VEHICLE_CONFIG, ...BMW_M5_2025_OVERRIDES } as VehicleConfig;

export default function App() {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const containerRef = useRef<HTMLDivElement | null>(null);

  // React states for UI / HUD
  const [config, setConfig] = useState<VehicleConfig>(INITIAL_CONFIG);
  const [activePresetKey, setActivePresetKey] = useState<string>(INITIAL_PRESET_KEY);
  const [currentColor, setCurrentColor] = useState<string>('#111827');
  const [cameraMode, setCameraMode] = useState<CameraMode>('chase');
  const [useMph, setUseMph] = useState<boolean>(false);
  const [showTelemetry, setShowTelemetry] = useState<boolean>(true);
  const [isTuningOpen, setIsTuningOpen] = useState<boolean>(false);
  const [isTestRunnerOpen, setIsTestRunnerOpen] = useState<boolean>(false);
  const [isMuted, setIsMuted] = useState<boolean>(false);
  const [activeKeys, setActiveKeys] = useState<{ [key: string]: boolean }>({});

  // Live vehicle telemetry state
  const [vehicleTelemetry, setVehicleTelemetry] = useState<VehicleState>(() => {
    const engine = new VehiclePhysicsEngine(INITIAL_CONFIG);
    return engine.state;
  });

  // Physics engine & rendering engine refs
  const physicsEngineRef = useRef<VehiclePhysicsEngine | null>(null);
  const carRendererRef = useRef<CarRenderer | null>(null);
  const envManagerRef = useRef<EnvironmentManager | null>(null);
  const cameraControllerRef = useRef<CameraController | null>(null);
  const keysDownRef = useRef<{ [code: string]: boolean }>({});
  const touchInputsRef = useRef<{
    throttle: boolean;
    brake: boolean;
    steerLeft: boolean;
    steerRight: boolean;
    handbrake: boolean;
  }>({
    throttle: false,
    brake: false,
    steerLeft: false,
    steerRight: false,
    handbrake: false,
  });

  // Initialize Three.js Scene and Main Simulation Loop
  useEffect(() => {
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;

    // 1. Three.js Core Setup
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x94a3b8);

    const width = container.clientWidth || window.innerWidth;
    const height = container.clientHeight || window.innerHeight;

    const camera = new THREE.PerspectiveCamera(62, width / height, 0.1, 2000);
    camera.position.set(0, 3.5, -7.5);

    const renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      powerPreference: 'high-performance',
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.05;

    // 2. Instantiate Systems
    const physicsEngine = new VehiclePhysicsEngine(config);
    physicsEngineRef.current = physicsEngine;

    const carRenderer = new CarRenderer(VEHICLE_PRESETS[activePresetKey]?.color || currentColor);
    carRendererRef.current = carRenderer;
    scene.add(carRenderer.rootGroup);

    const envManager = new EnvironmentManager(scene);
    envManagerRef.current = envManager;

    const cameraController = new CameraController(camera);
    cameraControllerRef.current = cameraController;

    // 3. Handle Canvas Resize
    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width: w, height: h } = entry.contentRect;
        if (w > 0 && h > 0) {
          camera.aspect = w / h;
          camera.updateProjectionMatrix();
          renderer.setSize(w, h);
        }
      }
    });
    resizeObserver.observe(container);

    // 4. Keyboard Event Listeners
    const handleKeyDown = (e: KeyboardEvent) => {
      keysDownRef.current[e.code] = true;
      setActiveKeys({ ...keysDownRef.current });

      // First user key interaction initializes audio context
      globalAudio.init();

      // Quick hotkeys
      if (e.code === 'KeyC') {
        const next = cameraController.nextMode();
        setCameraMode(next);
      } else if (e.code === 'KeyR') {
        physicsEngine.reset(0, 0, 0);
        envManager.resetCones();
      } else if (e.code === 'KeyT') {
        setShowTelemetry((prev) => !prev);
      } else if (e.code === 'KeyP') {
        setIsTuningOpen((prev) => !prev);
      } else if (e.code === 'KeyV') {
        // Toggle 3D Force Vectors
        physicsEngine.state.showForceVectors3D = !physicsEngine.state.showForceVectors3D;
      } else if (e.code === 'KeyF' || e.code === 'KeyX') {
        // Toggle DRS
        physicsEngine.toggleDrs();
      } else if (e.code === 'KeyJ' || e.code === 'KeyK') {
        // Trigger Clutch Kick
        physicsEngine.triggerClutchKick();
      } else if (e.code === 'KeyU') {
        const muted = globalAudio.toggleMute();
        setIsMuted(muted);
      } else if (e.code === 'KeyM') {
        physicsEngine.state.isAutomatic = !physicsEngine.state.isAutomatic;
      } else if (e.code === 'KeyB') {
        // Toggle ABS Mode
        const modes: ('OFF' | 'SPORT' | 'FULL')[] = ['OFF', 'SPORT', 'FULL'];
        const currentIdx = modes.indexOf(physicsEngine.config.absMode);
        const nextMode = modes[(currentIdx + 1) % modes.length];
        physicsEngine.config.absMode = nextMode;
        setConfig({ ...physicsEngine.config });
      } else if (e.code === 'KeyN') {
        // Toggle TCS Mode
        const modes: ('OFF' | 'SPORT' | 'FULL')[] = ['OFF', 'SPORT', 'FULL'];
        const currentIdx = modes.indexOf(physicsEngine.config.tcsMode);
        const nextMode = modes[(currentIdx + 1) % modes.length];
        physicsEngine.config.tcsMode = nextMode;
        setConfig({ ...physicsEngine.config });
      } else if (e.code === 'KeyY') {
        setIsTestRunnerOpen((prev) => !prev);
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      keysDownRef.current[e.code] = false;
      setActiveKeys({ ...keysDownRef.current });
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    // 5. Main Animation & Physics Loop
    let animationFrameId: number;
    let lastTime = performance.now();
    let hudUpdateTimer = 0;

    const animate = (currentTime: number) => {
      animationFrameId = requestAnimationFrame(animate);

      const deltaTime = Math.min((currentTime - lastTime) / 1000, 0.05);
      lastTime = currentTime;

      // Extract control inputs from keyboard or touch
      const keys = keysDownRef.current;
      const touches = touchInputsRef.current;

      const isThrottle = keys['KeyW'] || keys['ArrowUp'] || touches.throttle;
      const isBrake = keys['KeyS'] || keys['ArrowDown'] || touches.brake;
      const isLeft = keys['KeyA'] || keys['ArrowLeft'] || touches.steerLeft;
      const isRight = keys['KeyD'] || keys['ArrowRight'] || touches.steerRight;
      const isHandbrake = keys['Space'] || touches.handbrake;

      const throttleInput = isThrottle ? 1.0 : 0;
      const brakeInput = isBrake ? 1.0 : 0;
      const steerInput = (isLeft ? 1.0 : 0) - (isRight ? 1.0 : 0);

      const shiftUp = keys['ShiftLeft'] || keys['ShiftRight'];
      const shiftDown = keys['ControlLeft'] || keys['ControlRight'];

      // Step Multi-Substepped Physics Engine
      const state = physicsEngine.update(deltaTime, {
        throttle: throttleInput,
        brake: brakeInput,
        steer: steerInput,
        handbrake: isHandbrake,
        shiftUp,
        shiftDown,
      });

      // Update 3D Visuals & Dynamic Camber / Glowing Brakes / Backfires / 3D Force Vectors
      carRenderer.update(state, physicsEngine.config);

      // Update Proving Grounds Environment (Skid marks, smoke particles, cone collisions)
      envManager.update(deltaTime, state.x, state.z, state.yaw, state.speedMs, state.wheels);

      // Update Camera Tracking & FOV
      cameraController.update(deltaTime, state);

      // Update Audio Synthesizer (Engine harmonics, Turbo Whine, BOV Flutter, ABS Chatter, Tire Screech, Rev Limits)
      const maxSkid = Math.max(...state.wheels.map((w) => (w.isSkidding ? w.skidIntensity : 0)));
      const kerbRumble = Math.max(...state.wheels.map((w) => (w.surfaceType === 'kerb' ? 1.0 : 0)));
      globalAudio.update(
        state.rpm,
        physicsEngine.config.maxRpm,
        state.throttle,
        state.speedKmh,
        maxSkid,
        state.turboBoostPsi,
        state.turboBlowOff,
        state.absActive,
        state.isRevLimiting,
        state.revCutBounce,
        kerbRumble
      );

      // Render Three.js Scene
      renderer.render(scene, camera);

      // Throttle React State updates to ~30 FPS for optimal performance
      hudUpdateTimer += deltaTime;
      if (hudUpdateTimer >= 0.033) {
        hudUpdateTimer = 0;
        setVehicleTelemetry({ ...state });
      }
    };

    animationFrameId = requestAnimationFrame(animate);

    return () => {
      cancelAnimationFrame(animationFrameId);
      resizeObserver.disconnect();
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      renderer.dispose();
    };
  }, []);

  // Update physics config when changed in tuning modal or preset
  const handleConfigChange = (newConfig: VehicleConfig) => {
    setConfig(newConfig);
    if (physicsEngineRef.current) {
      physicsEngineRef.current.setConfig(newConfig);
    }
  };

  // Preset Selection
  const handleSelectPreset = (presetKey: string) => {
    const preset = VEHICLE_PRESETS[presetKey];
    if (!preset) return;
    setActivePresetKey(presetKey);
    setCurrentColor(preset.color);
    const mergedConfig: VehicleConfig = {
      ...DEFAULT_VEHICLE_CONFIG,
      ...preset.config,
    };
    handleConfigChange(mergedConfig);

    if (carRendererRef.current) {
      carRendererRef.current.setBodyColor(preset.color);
    }
  };

  // Color change
  const handleChangeColor = (hexColor: string) => {
    setCurrentColor(hexColor);
    if (carRendererRef.current) {
      carRendererRef.current.setBodyColor(hexColor);
    }
  };

  // Camera Switcher
  const handleNextCamera = () => {
    if (cameraControllerRef.current) {
      const next = cameraControllerRef.current.nextMode();
      setCameraMode(next);
    }
  };

  // Reset Car Position
  const handleResetCar = () => {
    if (physicsEngineRef.current) {
      physicsEngineRef.current.reset(0, 0, 0);
    }
    if (envManagerRef.current) {
      envManagerRef.current.resetCones();
    }
  };

  // Clear Skid Marks
  const handleClearSkidMarks = () => {
    if (envManagerRef.current) {
      envManagerRef.current.clearSkidMarks();
    }
  };

  // Toggle Mute Audio
  const handleToggleMute = () => {
    globalAudio.init();
    const muted = globalAudio.toggleMute();
    setIsMuted(muted);
  };

  // Touch controls callback
  const handleTouchInput = (action: 'throttle' | 'brake' | 'steerLeft' | 'steerRight' | 'handbrake', active: boolean) => {
    globalAudio.init();
    touchInputsRef.current[action] = active;
  };

  // 3D Force Vectors Toggle
  const handleToggleForceVectors = () => {
    if (physicsEngineRef.current) {
      physicsEngineRef.current.state.showForceVectors3D = !physicsEngineRef.current.state.showForceVectors3D;
    }
  };

  // Trigger Clutch Kick
  const handleTriggerClutchKick = () => {
    globalAudio.init();
    if (physicsEngineRef.current) {
      physicsEngineRef.current.triggerClutchKick();
    }
  };

  // Toggle DRS
  const handleToggleDrs = () => {
    if (physicsEngineRef.current) {
      physicsEngineRef.current.toggleDrs();
    }
  };

  return (
    <div
      ref={containerRef}
      id="driving-simulator-app"
      className="relative w-screen h-screen overflow-hidden bg-slate-950 select-none font-sans"
    >
      {/* 3D WebGL Canvas */}
      <canvas ref={canvasRef} className="absolute inset-0 w-full h-full block cursor-grab active:cursor-grabbing" />

      {/* Primary HUD & Telemetry */}
      <DashboardUI
        state={vehicleTelemetry}
        config={config}
        useMph={useMph}
        onToggleUnit={() => setUseMph(!useMph)}
        onToggleAuto={() => {
          if (physicsEngineRef.current) {
            physicsEngineRef.current.state.isAutomatic = !physicsEngineRef.current.state.isAutomatic;
          }
        }}
        showTelemetry={showTelemetry}
        onToggleAbs={() => {
          const modes: ('OFF' | 'SPORT' | 'FULL')[] = ['OFF', 'SPORT', 'FULL'];
          const currentIdx = modes.indexOf(config.absMode);
          const nextMode = modes[(currentIdx + 1) % modes.length];
          handleConfigChange({ ...config, absMode: nextMode });
        }}
        onToggleTcs={() => {
          const modes: ('OFF' | 'SPORT' | 'FULL')[] = ['OFF', 'SPORT', 'FULL'];
          const currentIdx = modes.indexOf(config.tcsMode);
          const nextMode = modes[(currentIdx + 1) % modes.length];
          handleConfigChange({ ...config, tcsMode: nextMode });
        }}
        onToggleForceVectors={handleToggleForceVectors}
        onTriggerClutchKick={handleTriggerClutchKick}
        onToggleDrs={handleToggleDrs}
      />

      {/* Quick Toolbar & Keyboard Visualizer */}
      <ControlsOverlay
        cameraMode={cameraMode}
        onNextCamera={handleNextCamera}
        onReset={handleResetCar}
        onClearSkidMarks={handleClearSkidMarks}
        isMuted={isMuted}
        onToggleMute={handleToggleMute}
        onOpenTuning={() => setIsTuningOpen(true)}
        onOpenTestRunner={() => setIsTestRunnerOpen(true)}
        showTelemetry={showTelemetry}
        onToggleTelemetry={() => setShowTelemetry(!showTelemetry)}
        activePresetKey={activePresetKey}
        onSelectPreset={handleSelectPreset}
        activeKeys={activeKeys}
        onTouchInput={handleTouchInput}
      />

      {/* Physics & Chassis Tuning Workshop Modal */}
      <TuningModal
        isOpen={isTuningOpen}
        onClose={() => setIsTuningOpen(false)}
        config={config}
        onSaveConfig={handleConfigChange}
        onSelectPreset={handleSelectPreset}
        currentColor={currentColor}
        onChangeColor={handleChangeColor}
      />

      {/* Physics 2.0 Headless Acceptance Test Runner Modal */}
      <PhysicsTestRunnerModal
        isOpen={isTestRunnerOpen}
        onClose={() => setIsTestRunnerOpen(false)}
        config={config}
      />
    </div>
  );
}
