# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# FIRST PRIORITY: EVERYTHING SHOULD BE PUBLIC
# STATIC WHENEVER POSSIBLE
# FINAL ONLY FOR CONSTANTS

## What this is

WPILib-based FRC 2026 robot code (Java 17) built around a **sense → schedule → actuate** loop. Uses CTRE Phoenix 6 motors, PhotonVision, AdvantageKit (junction) logging, and swerve drive kinematics/estimation.

## Build commands

```bash
# Build
./gradlew build

# Format all Java/Gradle/XML/Markdown files (run before committing)
./gradlew spotlessApply

# Deploy to RoboRIO
./gradlew deploy

# Run simulation (opens WPILib sim GUI)
./gradlew simulateJava

# Log replay watcher
./gradlew replayWatch
```

On Windows use `gradlew.bat` instead of `./gradlew`.

**Running from bash (Claude Code / WSL / Git Bash):** Gradle requires the WPILib JDK. Set `JAVA_HOME` before invoking gradlew:

```bash
JAVA_HOME="C:/Users/Public/wpilib/2026/jdk" PATH="C:/Users/Public/wpilib/2026/jdk/bin:$PATH" ./gradlew build
```

## Architecture

### Main loop (`RobotContainer.periodic()`)

```
1. intake.sense()        // read sensors
2. shooter.sense()
3. vision.sense()
4. CommandScheduler.run()  // execute commands
5. intake.actuate()      // write actuators
```

The drivetrain (`CommandSwerveDrivetrain`) is a `SubsystemBase` — its periodic/actuate logic runs inside `CommandScheduler.run()`, not explicitly in `RobotContainer.periodic()`.

All subsystems are `static` fields in `RobotContainer` and statically imported (`import static frc.robot.RobotContainer.*`) in most files. Do not create new subsystem instances — reference the statics.

### Key files

| File | Purpose |
|------|---------|
| [Robot.java](src/main/java/frc/robot/Robot.java) | Lifecycle + Logger setup (real vs. simulation/replay) |
| [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) | Static subsystem wiring, `init()`/`periodic()` order, bindings |
| [FieldConstants.java](src/main/java/frc/robot/FieldConstants.java) | Field layout, hub positions, AprilTag layout enum |
| [Constants.java](src/main/java/frc/robot/Constants.java) | CAN IDs only — gains/limits live in each subsystem class |
| [Util.java](src/main/java/frc/robot/Util.java) | `deadband`, `squareCurve`, `buildCov`, `flip`/`flipIfRed` helpers |
| [CommandSwerveDrivetrain.java](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java) | CTRE Phoenix 6 swerve (extends TunerSwerveDrivetrain) |
| [Vision.java](src/main/java/frc/robot/subsystem/vision/Vision.java) | PhotonPoseEstimator → drivetrain pose estimator fusion |
| [generated/TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java) | Module configs, CAN IDs, speed limits (Tuner X output — edit carefully) |

### Two coexisting subsystem patterns

**New pattern (`subsystem/`)** — AdvantageKit-based, use this for new code:
- HW class with `init()`, `sense(Inputs)`, `actuate(...)` hooks
- `Inputs` is an `@AutoLog`-annotated POJO; AdvantageKit generates `*AutoLogged`
- `sense()` calls `Logger.processInputs(path, inputs)`; `actuate()` calls `Logger.recordOutput(...)`
- Examples: `Vision.java` / `VisionHW.java` / `VisionInputs.java`

**Old pattern (`subsystems/`)** — direct motor control, used by intake/shooter/drivetrain:
- Methods call motors directly; selective `Logger.recordOutput()` sprinkled in
- Will be migrated to the new pattern over time

### Controls interface

`subsystem/controls/Controls.java` is an interface abstracting driver input. `XboxControls.java` implements it. `RobotContainer.controls` is the static instance — commands read from it instead of hardcoding joystick ports.

### Adding a new subsystem

1. Create a HW class (new pattern) with `init()`, `sense(Inputs)`, `actuate(...)` and an `@AutoLog` Inputs POJO.
2. Add a `static` instance to `RobotContainer` and call `init()` in `RobotContainer.init()`.
3. Add `sense()` before `CommandScheduler.run()` and `actuate()` after in `RobotContainer.periodic()`.
4. Use `/SubsystemName/...` Logger paths consistent with existing subsystems.

## Naming conventions

**Measured / target / setpoint distinction:**
- Sensor readings: short names with no prefix — `angle`, `vel`, `drivePos`, `state`
- Commanded from higher-level code: `target*` — `targetVel`, `targetAng`
- Controller/trapezoid outputs: `setpoint*` — `setpointVel`, `setpointAng`

**Math-style locals in algorithms/commands:**
- `v_x`, `v_y` (field-relative), `v` or `mag` for magnitude, `theta` for heading, `w` for angular velocity

**Constants:**
- `ALL_CAPS_SUBSYSTEM_kGain` — e.g., `DRIVE_kS`, `DRIVE_kV`, `TURN_kP`, `TURN_kD`
- Physical limits: `MAX_VEL` (m/s), `MAX_W` (rad/s)

**Trapezoid profiles:**
- `TrapezoidProfile.Constraints` as `static final` constants
- `PIDController` and `TrapezoidProfile` as non-static instance fields (they hold state)

## Scope / Static / Final patterns

**Subsystem instances:**
- All hardware subsystems are `static` fields in `RobotContainer` — initialized once at startup, accessible globally via static import.
- Do NOT create new instances of subsystems elsewhere in the codebase.

**Visibility (no encapsulation):**
- Subsystem and HW class fields: **`public`** by default. Direct access is preferred over getters — it simplifies data flow and logging.
- Command classes: `public` everywhere.

**Constants and configuration:**
- All constants: **`public static final`** — gains, limits, config arrays.
- Constants are defined in the HW/subsystem class where they're used, or in `Constants.java` (CAN IDs) / `FieldConstants.java` (field geometry).

**State holders (not final):**
- `PIDController`, `TrapezoidProfile`, pose estimators: **non-static instance fields**. They hold state and **must not be `final`** — they may be reset/reconfigured.
- Inputs POJOs: instance fields, auto-logged by AdvantageKit.

**Summary:**
- `static` = single instance, shared across code (subsystems, constants)
- `public` = default for fields (no encapsulation; direct access preferred)
- `final` = immutable reference for constants and configuration arrays
- Avoid `final` on stateful objects (controllers, estimators, profiles)

## Hardware config

- **Swerve module IDs, angle offsets, CAN bus name**: `TunerConstants.java` (generated by Tuner X) — edit there, not scattered through code. CAN bus is named `"Ducky"`. Pigeon 2 IMU is CAN ID 2.
- **Other CAN IDs** (intake, shooter, feeder): `Constants.java`.
- **AprilTag layout**: `AprilTagFields.k2026RebuiltWelded` enum (loaded at runtime in `FieldConstants.java`). No JSON files in deploy — the layout is bundled with WPILib.

## Subsystem details

### Drivetrain (`CommandSwerveDrivetrain`)

- Extends `TunerSwerveDrivetrain` (Phoenix 6 generated). Adds `SubsystemBase`, vision measurement injection, hub-distance math, and SysId routines.
- **Operator perspective** (field-relative zero heading) is set to 0° blue / 180° red. It resets automatically on disable and on first enable after alliance color is known.
- **Vision injection**: `addVisionMeasurement()` wraps the superclass call and converts FPGA timestamps with `Utils.fpgaToCurrentTime()` — don't skip this wrapper when adding measurements.
- **Hub math**: `getHubAngle()` and `getHubDistance()` return bearing/distance to the red alliance hub (`FieldConstants.redAllianceHub`). These feed shooter aiming.
- **SysId**: Three built-in routines (translation, steer, rotation). Triggered via combo button presses — Back+Y/X for quasistatic, Start+Y/X for dynamic.
- While **disabled**, the drivetrain applies an idle `SwerveRequest` so the robot can be pushed freely.

### Intake (`IntakeSubsystem`)

- **Feeder motor (CAN ID 60) lives in IntakeSubsystem**, not ShooterSubsystem. This is intentional — it's physically part of the intake path.
- **Extension** uses two TalonFX motors (IDs 0 + 21) in follower/opposed configuration with a 12:1 gear reduction (`EXTEND_RATIO = 1/12`).
  - Controlled with `PositionVoltage` (slot 0) and a trapezoid profile: 12 rot/s, 30 rot/s².
  - kP is multiplied by `EXTEND_RATIO × 2π` during `init()` to account for gear ratio and unit conversion — the gain in the source constant is **not** the value sent to the motor.
  - Position targets: 0.0 rot (retracted) → 10.0 rot (extended).
  - `extendStop()` freezes the setpoint at the current position for mid-motion halts.

### Shooter (`ShooterSubsystem`)

- Flywheel (IDs 62 + 61, follower/opposed) uses `VelocityDutyCycle` — setpoint is in RPS.
- Elevator (ID 32) is **open-loop** `DutyCycleOut`. Negative speed moves it up.
- `Targetting.java` holds the `SHOT_TABLE` (distance → RPM, elevator position, time-of-flight) and performs linear interpolation via `shotSpeeds()` — tune all values before match use.

### Alliance / field coordinate system

- `RobotContainer.isRed()` uses `DriverStation.getAlliance()` and **defaults to blue** if no alliance is set.
- `Util.flipIfRed(Pose2d/Translation2d/Rotation2d/double)` mirrors coordinates across the field center (adds π to angles, reflects x/y). Call this wherever field-relative targets differ by alliance.

## Logging

Uses AdvantageKit (junction). Simulation runs in replay mode by default using `WPILOGReader` (see `Robot.robotInit()`). Log liberally — sensor readings, setpoints, and control outputs — to enable post-match replay and debugging.

## Drive command

`FieldOrientedDrive` (default command on drivetrain) decomposes the left stick into **polar coordinates** before applying deadband — deadband is applied to the magnitude, not raw x/y. This prevents diagonal bias at low speeds. Right stick X controls rotation. All values are squared for sensitivity.

## Vision / pose estimation notes

- Pose estimator covariance is set in `Vision.java` (standard deviations) and `CommandSwerveDrivetrain` constructor — don't duplicate it elsewhere.
- Vision uses `result.getTimestampSeconds()` from PhotonVision results for measurement timestamps; these are then converted via `Utils.fpgaToCurrentTime()` in `CommandSwerveDrivetrain.addVisionMeasurement()`.
- **Two fusion strategies run in parallel:**
  - `pnpDistTrig` (single-tag): runs per-tag, weights by 1/d², σ²_xy = 0.04/Σ(1/d²). Active by default.
  - `multiTag` (≥2 tags): uses onboard multi-tag solve; loose XY covariance, tight theta. Useful for heading correction across the field.
  - Both can be toggled at runtime via `LoggedNetworkBoolean` flags.
- **Sanity check**: poses outside field bounds (0–26.29 m × 0–13.39 m) or with Z outside −2.0–3.0 m are rejected.
- **Camera placeholder**: `VisionHW` camera name and calibration JSON are currently `"TODO"` — replace with real camera name and deploy the corresponding calibration JSON before using vision in matches.
- `FieldConstants.FIELD` is currently `WELDED` (2026 Rebuilt Welded field).
- Alliance flipping: use `Util.flipIfRed(...)` — it checks `RobotContainer.isRed()` and applies field-relative coordinate flips.
