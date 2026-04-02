# RobotCode2026 - Team Koi 6230 FRC Robot

Team Koi's 2026 FRC robot codebase, built on the KoiUpstream framework.

## Overview

This is a Java-based robot control system using WPILib 2026 with a swerve drivetrain, intake system, shooter mechanism, and vision processing. The code follows AdvantageKit's IO pattern for hardware abstraction and simulation support.

## Architecture

### Core Framework
- **KoiUpstream** (v1.0.28) - Team 6230's custom framework providing superstate management and upstream subsystems
- **AdvantageKit** (v26.0.2) - Logging, replay, and simulation infrastructure
- **WPILib 2026** - Core FRC libraries

### Control Flow
The robot uses a **superstate pattern** managed by `Superstate.getInstance()`:
- States: `IDLE`, `HOME`, `INTAKING`, `PREPARING_SHOOTER`, `PREPARING_SHOOTER_AND_INTAKING`, `SHOOTING`, `SHOOTING_AND_INTAKING`, `UNJAM`
- Each subsystem registers behavior callbacks for each superstate
- Superstate logic runs in `Robot.robotPeriodic()` at 50Hz

## Subsystems

### Drive (`Drive`)
Swerve drivetrain with 4 modules (FL, FR, BL, BR):
- Field-relative and robot-relative driving
- PathPlanner auto navigation integration
- SysId characterization support
- Aiming PID for orientation locking
- Vision pose estimation fusion
- Two drive modes: `defaultDrive` and `shootingDrive` (aims using ballistics calculator)

### Intake (`Intake`)
Pivoting intake with roller:
- Pivot angle control via profiled motion (kP, kI, kD, kS, kG, kV, kA)
- Conditional roller activation based on pivot angle
- Index cycling for shooting (steps through angles)
- Unjam mode with reverse roller

### Shooter (`Shooter`)
Flywheel shooter with hood and feeder roller:
- Ballistics calculator for trajectory-based setpoints
- Tunable PIDF + feedforward (kS, kV, kA)
- Hood servo positioning
- Feeder roller for note feeding
- Ready detection based on RPM tolerance

### Vision (`Vision`)
AprilTag-based pose estimation using Limelight:
- MegaTag1 (MT1) for yaw correction
- MegaTag2 (MT2) for full pose updates
- Dynamic standard deviation calculation based on tag count/distance
- Rejection logic for high gyro rates and large pose deltas

## Key Utilities

| Class | Purpose |
|-------|---------|
| `SwerveInputStream` | Controller input processing for swerve |
| `KoiController` | Custom controller wrapper |
| `AllianceFlipUtil` | Field-side flipping utilities |
| `MathHelper` | Math utilities |
| `LocalADStarAK` | PathPlanner pathfinding implementation |
| `Roller` / `RollerIO` | Generic roller mechanism abstraction |

## Build & Deploy

### Requirements
- Java 17
- WPILib 2026
- GradleRIO

### Commands
```bash
./gradlew build          # Build robot code
./gradlew deploy         # Deploy to robot
./gradlew simulateJava   # Run simulation with GUI
./gradlew replayWatch    # View AdvantageKit logs
```

## Configuration Files

| File | Purpose |
|------|---------|
| `RobotConstants.java` | Global constants (sim/replay mode, loop period) |
| `DriveConstants.java` | Swerve kinematics, speeds, PID gains |
| `IntakeConstants.java` | Intake angles, speeds, PID gains |
| `ShooterConstants.java` | Flywheel/hood parameters |
| `VisionConstants.java` | Vision thresholds, stddevs |
| `RobotMap.java` | CAN bus IDs, DIO ports |
| `pathplanner/settings.json` | Auto navigation config |
| `apriltags/*.json` | AprilTag field layouts |

## Controller Mapping

| Input | Action |
|-------|--------|
| Left Trigger | Intake |
| Right Trigger | Shoot |
| Right Bumper | Prepare shooter |
| A Button | Home |
| POV Up | Unjam |
| Left Bumper | Toggle orientation lock |

## Simulation

The codebase supports full simulation:
- All subsystems have `Sim` IO implementations
- Physics simulation for swerve, flywheel, and pivot mechanisms
- AdvantageKit log replay support

## Logging

AdvantageKit logs all inputs/outputs to:
- NetworkTables (NT4) for real-time viewing
- WPILOG files for replay analysis
- URCL for compact logging on real robot
