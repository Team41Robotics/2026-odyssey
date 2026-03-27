package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.autos.Autos;
import frc.robot.commands.drive.FieldOrientedDrive;
import frc.robot.commands.intake.ExtendIn;
import frc.robot.commands.intake.ExtendOut;
import frc.robot.commands.shooter.Align;
import frc.robot.commands.shooter.Shoot;
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

	public static AutoFactory autoFactory;
	public static AutoChooser autoChooser;
	public static Command autonomousCommand = null;

	public static void init() {
		intake.init();
		shooter.init();
		indexer.init();
		vision.init();

		drive.setDefaultCommand(new FieldOrientedDrive());
		shooter.setDefaultCommand(new ShooterTrack());

		Autos.init();
		autoFactory =
				new AutoFactory(drive::getPose, drive::setPose, Autos::choreoController, RobotContainer.isRed(), drive);
		autoChooser = new AutoChooser();
		// Register Choreo routines here, e.g.:
		// autoChooser.addRoutine("MyAuto", () -> autoFactory.newRoutine("myTrajectory"));
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autonomousCommand = autoChooser.selectedCommandScheduler();

		configureBindings();
	}

	public static void periodic() {
		Logger.recordOutput("MatchTime", DriverStation.getMatchTime());
		Logger.recordOutput("Enabled", DriverStation.isEnabled());
		Logger.recordOutput("Autonomous", DriverStation.isAutonomous());
		Logger.recordOutput("Teleop", DriverStation.isTeleop());
		Logger.recordOutput("IsRed", isRed());
		Logger.recordOutput("DSAttached", DriverStation.isDSAttached());
		Logger.recordOutput("FMSAttached", DriverStation.isFMSAttached());

		intake.sense();
		indexer.sense();
		shooter.sense();
		vision.sense();

		CommandScheduler.getInstance().run();

		intake.actuate();
		indexer.actuate();
		shooter.actuate();
	}

	private static void configureBindings() {
		// X-lock: point all wheels inward to resist pushing, brake on hold, coast on release
		controls.xLock().onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
		controls.xLock().onFalse(Commands.runOnce(() -> drive.setBrakeMode(false)));
		controls.xLock().whileTrue(new RunCommand(drive::stopWithX, drive));

		controls.align().whileTrue(new Align()); // button 2: auto-aim heading
		controls.shoot().whileTrue(new Shoot()); // button 1: flywheel + feeder


		controls.extendOut().whileTrue(new ExtendOut());
		controls.extendIn().whileTrue(new ExtendIn());

		// controls.sysidQuasiForward().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
		// controls.sysidQuasiBackward().whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
		// controls.sysidDynaForward().whileTrue(drive.sysIdDynamic(Direction.kForward));
		// controls.sysidDynaBackward().whileTrue(drive.sysIdDynamic(Direction.kReverse));
	}

	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}
}
