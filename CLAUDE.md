# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

FRC 2026 season robot code ("Odyssey"). Java 17, WPILib GradleRIO 2026, AdvantageKit logging, CTRE Phoenix 6 swerve, PhotonVision, ChoreoLib autos.

## Common commands

Use the Gradle wrapper (`./gradlew` on Unix, `gradlew.bat` on Windows):

- `./gradlew build` — compile + run Spotless checks + run tests.
- `./gradlew deploy` — deploy to roboRIO (team number from `.wpilib/wpilib_preferences.json`).
- `./gradlew simulateJava` — run desktop simulation (SimGUI + Driver Station auto-enabled, see `build.gradle`).
- `./gradlew spotlessApply` — auto-format. Spotless uses Palantir Java format, tab indent (4 spaces→tabs), excludes `**/choreo/**` and `BuildConstants.java`. CI/`build` will fail on unformatted code.
- `./gradlew test` — JUnit 5 tests only.
- `./gradlew test --tests "FullyQualifiedClass.method"` — run a single test.
- `./gradlew replayWatch` — AdvantageKit replay watcher (`org.littletonrobotics.junction.ReplayWatch`).

Desktop support is off (`includeDesktopSupport = false` in `build.gradle`) — simulation still works via `simulateJava`, but pure desktop runs aren't wired up.

## Architecture

### Entry points
`Main.java` → `Robot` (extends `LoggedRobot`, AdvantageKit). `Robot.robotInit` configures AdvantageKit data receivers (NT4 + WPILOG to `/U/logs` on real robot; replay source in sim), starts `Logger`, then calls `RobotContainer.init()`. `Robot.robotPeriodic` delegates to `RobotContainer.periodic()` and logs heap/GC stats.

### RobotContainer is the god object
[RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) holds **all subsystems as `public static` fields** and owns the whole periodic loop. Subsystems are accessed across the codebase via `import static frc.robot.RobotContainer.*;` — this is the established pattern, don't refactor toward DI.

`RobotContainer.periodic()` runs in a strict phased order every 20 ms (`LOOP_PERIOD = 0.020`):
1. `updateMatchPeriod()` + match-state logging
2. `subsystem.sense()` on each subsystem (reads hardware inputs, processes AdvantageKit inputs, reacts to dashboard buttons)
3. `CommandScheduler.getInstance().run()` (commands mutate subsystem target fields)
4. `subsystem.actuate()` on each subsystem (writes motor outputs)

**Commands must never touch hardware directly** — they set target fields (e.g., `shooter.targetFlywheelRPM`, `intake.targetPivotVoltage`) and `actuate()` applies them. Preserve this sense→schedule→actuate ordering when adding subsystems.

### Subsystem pattern (IO abstraction + AutoLog)
Each subsystem under `src/main/java/frc/robot/subsystem/<name>/` follows:
- `XxxInputs.java` — `@AutoLog`-annotated POJO of sensor values. The annotation processor generates `XxxInputsAutoLogged` at build time (do not hand-write it).
- `XxxHW.java` — hardware wrapper: `init()`, `sense(inputs)`, `actuate(inputs, ...targets)`.
- `XxxSubsystem.java` — extends `SubsystemBase`, owns `hw` + `inputs` + target-value fields, exposes `init/sense/actuate`. Logging goes through `Logger.processInputs("/Subsystem", inputs)` and `Logger.recordOutput(...)`.

Drive follows the same shape but with swappable IO layers: `GyroIO`/`GyroIOPigeon2`, `ModuleIO`/`ModuleIOTalonFX`, plus `PhoenixOdometryThread` for high-rate odometry. `TunerConstants.java` is generated from the CTRE Swerve Tuner (`tuner-project.json`) — regenerate rather than hand-editing.

### Controls
[Controls.java](src/main/java/frc/robot/subsystem/controls/Controls.java) is an interface; `JoystickControls` and `XboxControls` implement it. The active impl is chosen by the static field `RobotContainer.controls`. Add new inputs to the interface and both impls.

### Autos (Choreo)
`commands/autos/Autos.java` defines `AutoRoutine`s built on `autoFactory`. `Autos.choreoController` is a PID+feedforward swerve trajectory follower (tune `xController`/`yController`/`thetaController` there). Trajectories are referenced through `ChoreoTraj` constants in `src/main/java/frc/robot/choreo/` — **that directory is Choreo-codegen output, don't edit by hand** (also excluded from Spotless). Register routines in `RobotContainer.init()` via `autoChooser.addRoutine(...)`.

### Match-period state machine
`RobotContainer.updateMatchPeriod()` / `isHubActive()` encode a game-specific alliance-hub ownership schedule based on `DriverStation.getMatchTime()` and `getGameSpecificMessage()` (first char `R`/`B` picks who won auto). This drives `allianceHubStatus` + shift labels published to SmartDashboard — assume it's load-bearing for driver feedback, not dead code.

### Vendor deps
`vendordeps/`: AdvantageKit, Phoenix 6, ChoreoLib 2026, WPILib New Commands, PhotonLib. Add new vendor libs via their online JSON URL through WPILib VS Code / `./gradlew vendordep`.
