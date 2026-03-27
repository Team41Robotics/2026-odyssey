package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.drive.FieldOrientedDrive;
import frc.robot.commands.intake.ExtendIn;
import frc.robot.commands.intake.ExtendOut;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShooterTrack;
import frc.robot.subsystem.controls.Controls;
import frc.robot.subsystem.controls.XboxControls;
import frc.robot.subsystem.drive.Drive;
import frc.robot.subsystem.drive.GyroIOPigeon2;
import frc.robot.subsystem.drive.ModuleIOTalonFX;
import frc.robot.subsystem.drive.TunerConstants;
import frc.robot.subsystem.intake.IntakeSubsystem;
import frc.robot.subsystem.shooter.ShooterSubsystem;
import frc.robot.subsystem.vision.Vision;

public class RobotContainer {
	public static final double LOOP_PERIOD = 0.020;

	public static Controls controls = new XboxControls();
	public static Robot robot;

	public static Drive drive = new Drive(
			new GyroIOPigeon2(),
			new ModuleIOTalonFX(TunerConstants.FrontLeft),
			new ModuleIOTalonFX(TunerConstants.FrontRight),
			new ModuleIOTalonFX(TunerConstants.BackLeft),
			new ModuleIOTalonFX(TunerConstants.BackRight));
	public static IntakeSubsystem intake = new IntakeSubsystem();
	public static ShooterSubsystem shooter = new ShooterSubsystem();
	public static Vision vision = new Vision();

	public static AutoFactory autoFactory;
	public static AutoChooser autoChooser;
	public static Command autonomousCommand = null;

	public static void init() {
		intake.init();
		shooter.init();
		vision.init();

		drive.setDefaultCommand(new FieldOrientedDrive());
		shooter.setDefaultCommand(new ShooterTrack());

		autoFactory = new AutoFactory(
				drive::getPose,
				drive::setPose,
				(SwerveSample sample) -> drive.runVelocity(sample.getChassisSpeeds()),
				RobotContainer.isRed(),
				drive);
		autoChooser = new AutoChooser();
		// Register Choreo routines here, e.g.:
		// autoChooser.addRoutine("MyAuto", () -> autoFactory.newRoutine("myTrajectory"));
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autonomousCommand = autoChooser.selectedCommandScheduler();

		configureBindings();
	}

	public static void periodic() {
		intake.sense();
		shooter.sense();
		vision.sense();

		CommandScheduler.getInstance().run();

		intake.actuate();
		shooter.actuate();
	}

	private static void configureBindings() {
		// X-lock: point all wheels inward to resist pushing
		controls.xLock().whileTrue(new RunCommand(drive::stopWithX, drive));

		// Shoot on the fly: auto-aim + fire feeder when onTarget
		controls.fullShootBumper().whileTrue(new Shoot());

		// Manual elevator adjustment
		controls.elevatorUp()
				.whileTrue(new RunCommand(() -> shooter.setElevatorSpeed(-0.7), shooter)
						.finallyDo(() -> shooter.setElevatorSpeed(0.0)));

		controls.extendOut().onTrue(new ExtendOut());
		controls.extendIn().onTrue(new ExtendIn());

		// Manual feeder override
		controls.feederButton()
				.whileTrue(new RunCommand(() -> intake.setFeederSpeed(0.5)).finallyDo(() -> intake.setFeederSpeed(0)));

		controls.sysidQuasiForward().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
		controls.sysidQuasiBackward().whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
		controls.sysidDynaForward().whileTrue(drive.sysIdDynamic(Direction.kForward));
		controls.sysidDynaBackward().whileTrue(drive.sysIdDynamic(Direction.kReverse));
	}

	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}
}
