# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Deploy Commands

```bash
# Build the project
./gradlew build

# Deploy to the RoboRIO
./gradlew deploy

# Run simulation
./gradlew simulateJava

# Check code formatting (Google Java Format, AOSP style)
./gradlew spotlessCheck

# Auto-fix formatting
./gradlew spotlessApply

# Run tests
./gradlew test

# Replay log analysis
./gradlew replayWatch
```

## Project Overview

FRC Team 3647 (Millennium Falcons) 2026 robot code. Java 17, WPILib GradleRIO, AdvantageKit logging framework.

## Architecture

### Package Structure

- `team3647.frc2026.robot` — Robot entry point, `RobotContainer` (wires everything together)
- `team3647.frc2026.subsystems` — Hardware subsystems (SwerveDrive, Shooter, Indexer, Hood, Wrist, Rollers, Kicker, Climb, LEDs)
- `team3647.frc2026.subsystems.Swerve` — SwerveDrive interface with `SwerveDriveReal` and `SwerveDriveSim` implementations
- `team3647.frc2026.commands` — Command factories organized per-subsystem
- `team3647.frc2026.autos` — Autonomous routines using Choreo trajectories
- `team3647.frc2026.constants` — All hardware IDs, PID gains, and configuration constants
- `team3647.frc2026.Util` — AutoDrive (shooting calculations), RobotTracker (pose tracking), TargetingUtil (ballistics)
- `team3647.lib` — Shared library code (Team 254 geometry, Team 6328 VirtualSubsystem, motor drivers, vision)

### Key Patterns

- **Subsystems** use `PeriodicSubsystem` interface (Team 254 pattern) with `readPeriodicInputs()`/`writePeriodicOutputs()` separation
- **VirtualSubsystem** (Team 6328) for non-hardware subsystems like AutoDrive and RobotTracker, called via `VirtualSubsystem.periodicAll()`
- **Superstructure** orchestrates multi-subsystem actions (intake, prep, shoot, feed) as a state machine
- **RobotContainer** is the dependency injection hub — instantiates subsystems, binds controls, configures autos
- **Motor abstractions**: `TalonFXSubsystem` (CTRE Phoenix 6) and `SparkMaxFactory` (Rev)
- **AdvantageKit** `@AutoLog` annotations on IO classes for automatic telemetry

### Hardware

- **Swerve drive**: CTRE Phoenix 6 TalonFX motors on `"drive"` CANBus
- **Subsystem motors**: TalonFX + SparkMax on `"subsystems"` CANBus
- **Vision**: 3 Limelight cameras (left, right, mid) with AprilTag processing
- **Controllers**: Main joystick (port 0) + co-controller (port 1)

### Autonomous

Choreo trajectories in `src/main/deploy/choreo/`. Alliance-aware path selection (red/blue auto lists). Autonomous modes: Preload, Mid Rush, Double Mid Rush, Double Mid Rush Loop.

## Code Style

Google Java Format with AOSP style, enforced by Spotless. CI runs `spotlessCheck` on every push. Run `./gradlew spotlessApply` before committing.
