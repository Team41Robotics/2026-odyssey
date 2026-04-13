package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MatchAlerts;
import frc.robot.commands.PreMatchCheck;
import frc.robot.commands.autos.Autos;
import frc.robot.commands.drive.FieldOrientedDrive;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeUp;
import frc.robot.commands.intake.ZeroPivot;
import frc.robot.commands.shooter.AlignTeleop;
import frc.robot.commands.shooter.ShootTeleop;
import frc.robot.commands.shooter.ShooterTrack;
import frc.robot.subsystem.controls.Controls;
import frc.robot.subsystem.controls.JoystickControls;
import frc.robot.subsystem.drive.Drive;
import frc.robot.subsystem.drive.GyroIOPigeon2;
import frc.robot.subsystem.drive.ModuleIOTalonFX;
import frc.robot.subsystem.drive.TunerConstants;
import frc.robot.subsystem.indexer.IndexerSubsystem;
import frc.robot.subsystem.intake.IntakeSubsystem;
import frc.robot.subsystem.shooter.ShooterSubsystem;
import frc.robot.subsystem.vision.Vision;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class RobotContainer {
	public static final double LOOP_PERIOD = 0.020;

	// public static Controls controls = new XboxControls();
	public static Controls controls = new JoystickControls();
	public static Robot robot;

	public static Drive drive = new Drive(
			new GyroIOPigeon2(),
			new ModuleIOTalonFX(TunerConstants.FrontLeft),
			new ModuleIOTalonFX(TunerConstants.FrontRight),
			new ModuleIOTalonFX(TunerConstants.BackLeft),
			new ModuleIOTalonFX(TunerConstants.BackRight));
	public static IntakeSubsystem intake = new IntakeSubsystem();
	public static ShooterSubsystem shooter = new ShooterSubsystem();
	public static IndexerSubsystem indexer = new IndexerSubsystem();
	public static Vision vision = new Vision();

	public static LoggedNetworkBoolean zeroPivotButton = new LoggedNetworkBoolean("/Intake/zeroPivot", false);

	public static String currentPeriod = "DISABLED";
	public static double periodTimeRemaining = 0;
	public static String allianceHubStatus = "Unknown";

	public static void init() {
		intake.init();
		shooter.init();
		indexer.init();
		vision.init();

		drive.setDefaultCommand(new FieldOrientedDrive());
		shooter.setDefaultCommand(new ShooterTrack());
		intake.setDefaultCommand(new IntakeDown());

		Autos.init();

		CommandScheduler.getInstance().schedule(new PreMatchCheck());
		CommandScheduler.getInstance().schedule(new MatchAlerts());

		configureBindings();
	}

	public static void periodic() {
		updateMatchPeriod();
		Logger.recordOutput("MatchTime", DriverStation.getMatchTime());
		Logger.recordOutput("Enabled", DriverStation.isEnabled());
		Logger.recordOutput("Autonomous", DriverStation.isAutonomous());
		Logger.recordOutput("Teleop", DriverStation.isTeleop());
		Logger.recordOutput("IsRed", isRed());
		Logger.recordOutput("DSAttached", DriverStation.isDSAttached());
		Logger.recordOutput("FMSAttached", DriverStation.isFMSAttached());

		long tIntakeSense = RobotController.getFPGATime();
		intake.sense();
		Logger.recordOutput("Timing/intake_sense_us", RobotController.getFPGATime() - tIntakeSense);

		long tIndexerSense = RobotController.getFPGATime();
		indexer.sense();
		Logger.recordOutput("Timing/indexer_sense_us", RobotController.getFPGATime() - tIndexerSense);

		long tShooterSense = RobotController.getFPGATime();
		shooter.sense();
		Logger.recordOutput("Timing/shooter_sense_us", RobotController.getFPGATime() - tShooterSense);

		long tVisionSense = RobotController.getFPGATime();
		vision.sense();
		Logger.recordOutput("Timing/vision_sense_us", RobotController.getFPGATime() - tVisionSense);

		long tScheduler = RobotController.getFPGATime();
		CommandScheduler.getInstance().run();
		Logger.recordOutput("Timing/scheduler_us", RobotController.getFPGATime() - tScheduler);

		Autos.autoChooser.periodic();

		long tIntakeActuate = RobotController.getFPGATime();
		intake.actuate();
		Logger.recordOutput("Timing/intake_actuate_us", RobotController.getFPGATime() - tIntakeActuate);

		long tIndexerActuate = RobotController.getFPGATime();
		indexer.actuate();
		Logger.recordOutput("Timing/indexer_actuate_us", RobotController.getFPGATime() - tIndexerActuate);

		long tShooterActuate = RobotController.getFPGATime();
		shooter.actuate();
		Logger.recordOutput("Timing/shooter_actuate_us", RobotController.getFPGATime() - tShooterActuate);
	}

	private static void configureBindings() {
		// X-lock: point all wheels inward to resist pushing, brake on hold, coast on release
		controls.xLock().onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
		controls.xLock().onFalse(Commands.runOnce(() -> drive.setBrakeMode(false)));
		controls.xLock().whileTrue(new RunCommand(drive::stopWithX, drive));

		controls.align().whileTrue(new AlignTeleop());
		controls.shoot().whileTrue(new ShootTeleop());

		controls.extendOut().whileTrue(new IntakeDown());
		controls.extendIn().whileTrue(new IntakeUp());

		new Trigger(zeroPivotButton::get).onTrue(new ZeroPivot());

		controls.invertToggle().onTrue(Commands.runOnce(() -> {
			JoystickControls.inverted = !JoystickControls.inverted;
			Logger.recordOutput("Drive/Inverted", JoystickControls.inverted);
		}));

		// controls.sysidQuasiForward().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
		// controls.sysidQuasiBackward().whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
		// controls.sysidDynaForward().whileTrue(drive.sysIdDynamic(Direction.kForward));
		// controls.sysidDynaBackward().whileTrue(drive.sysIdDynamic(Direction.kReverse));
	}

	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}

	public static boolean redWonAuto() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData != null && gameData.length() > 0) {
			switch (gameData.charAt(0)) {
				case 'R':
					return true;
				case 'B':
					return false;
				default:
					break;
			}
		}
		return false;
	}

	public static boolean isHubActive() {
		if (DriverStation.isAutonomousEnabled()) {
			return true;
		}
		if (!DriverStation.isTeleopEnabled()) {
			return false;
		}
		double matchTime = DriverStation.getMatchTime();
		boolean weWonAuto = (isRed() && redWonAuto()) || (!isRed() && !redWonAuto());

		if (matchTime > 130) {
			return true;
		} else if (matchTime > 105) {
			return !weWonAuto;
		} else if (matchTime > 80) {
			return weWonAuto;
		} else if (matchTime > 55) {
			return !weWonAuto;
		} else if (matchTime > 30) {
			return weWonAuto;
		} else {
			return true;
		}
	}

	public static void updateMatchPeriod() {
		double matchTime = DriverStation.getMatchTime();

		if (DriverStation.isDisabled()) {
			currentPeriod = "DISABLED";
			periodTimeRemaining = 0;
			allianceHubStatus = "#000000";
		} else if (DriverStation.isAutonomous()) {
			currentPeriod = "AUTO";
			periodTimeRemaining = matchTime;
			allianceHubStatus = "#FF00FF";
		} else if (DriverStation.isTeleop()) {
			boolean weWonAuto = (isRed() && redWonAuto()) || (!isRed() && !redWonAuto());

			if (matchTime > 130) {
				currentPeriod = "TRANSITION";
				periodTimeRemaining = matchTime - 130;
				allianceHubStatus = weWonAuto ? "#FF0000" : "#00FF00";
			} else if (matchTime > 105) {
				currentPeriod = "SHIFT 1";
				periodTimeRemaining = matchTime - 105;
				allianceHubStatus = weWonAuto ? "#FF0000" : "#00FF00";
			} else if (matchTime > 80) {
				currentPeriod = "SHIFT 2";
				periodTimeRemaining = matchTime - 80;
				allianceHubStatus = weWonAuto ? "#00FF00" : "#FF0000";
			} else if (matchTime > 55) {
				currentPeriod = "SHIFT 3";
				periodTimeRemaining = matchTime - 55;
				allianceHubStatus = weWonAuto ? "#FF0000" : "#00FF00";
			} else if (matchTime > 30) {
				currentPeriod = "SHIFT 4";
				periodTimeRemaining = matchTime - 30;
				allianceHubStatus = weWonAuto ? "#00FF00" : "#FF0000";
			} else {
				currentPeriod = "END GAME";
				periodTimeRemaining = matchTime;
				allianceHubStatus = "#00FFFF";
			}
		} else {
			currentPeriod = "TEST";
			periodTimeRemaining = 0;
			allianceHubStatus = "#000000";
		}

		SmartDashboard.putString("MatchPeriod", currentPeriod);
		SmartDashboard.putNumber("PeriodTimeRemaining", periodTimeRemaining);
		SmartDashboard.putString("AllianceHubStatus", allianceHubStatus);
		SmartDashboard.putBoolean("RedWonAuto", redWonAuto());
		SmartDashboard.putString("GameData", DriverStation.getGameSpecificMessage());
	}
}
