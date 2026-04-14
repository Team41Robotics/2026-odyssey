package frc.robot.commands.autos;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LoggedAutoChooser;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeUp;
import frc.robot.commands.shooter.AlignTeleop;
import frc.robot.commands.shooter.Shoot;
import org.littletonrobotics.junction.Logger;

public class Autos {
	public static PIDController xController = new PIDController(15.0, 0, 0);
	public static PIDController yController = new PIDController(15.0, 0, 0);
	public static PIDController thetaController = new PIDController(10, 0, 0);

	public static AutoFactory autoFactory;
	public static LoggedAutoChooser autoChooser;
	public static Command autonomousCommand;

	public static void init() {
		thetaController.enableContinuousInput(-PI, PI);

		autoFactory = new AutoFactory(drive::getPose, drive::setPose, Autos::choreoController, true, drive);
		autoChooser = new LoggedAutoChooser("AutoChooser");
		autoChooser.addCmd("Do Nothing", Commands::none);
		autoChooser.addCmd(
				"Drive Forward 1s",
				() -> Commands.sequence(
						Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), drive)
								.withTimeout(1.0),
						Commands.runOnce(drive::stop, drive)));
		autoChooser.addRoutine("Test Path (Loop)", () -> {
			var routine = autoFactory.newRoutine("Test Path (Loop)");
			var traj = ChoreoTraj.test.asAutoTraj(routine);
			Logger.recordOutput(
					"/Auto/Test/trajectory",
					traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active().onTrue(traj.cmd());
			traj.done().onTrue(traj.cmd());
			return routine;
		});
		autoChooser.addRoutine("Trench1", () -> {
			var routine = autoFactory.newRoutine("Trench1");
			var traj = ChoreoTraj.Trench1.asAutoTraj(routine);
			Logger.recordOutput(
					"/Auto/Trench1/trajectory",
					traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active()
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "started")),
							Commands.parallel(traj.cmd(), new IntakeUp())));
			traj.atTime("DeployIntake")
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "deployIntake")),
							new IntakeDown()));
			traj.done()
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "aligning")),
							new AlignTeleop().withTimeout(0.5),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "firing")),
							new Shoot().withTimeout(3.0),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "done"))));
			return routine;
		});
		autonomousCommand = autoChooser.selectedCommandScheduler();
	}

	public static void choreoController(SwerveSample sample) {
		Pose2d pose = drive.getPose();

		double xff = sample.vx;
		double yff = sample.vy;
		double wff = sample.omega;

		double xfb = xController.calculate(pose.getX(), sample.x);
		double yfb = yController.calculate(pose.getY(), sample.y);
		double wfb = thetaController.calculate(pose.getRotation().getRadians(), sample.heading);

		ChassisSpeeds speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(xff + xfb, yff + yfb, wff + wfb, pose.getRotation());

		Logger.recordOutput("/Auto/currentPose", pose);
		Logger.recordOutput("/Auto/targetPose", sample.getPose());
		Logger.recordOutput("/Auto/targetVxMps", sample.vx);
		Logger.recordOutput("/Auto/targetVyMps", sample.vy);
		Logger.recordOutput("/Auto/targetOmegaRadPerSec", sample.omega);
		Logger.recordOutput("/Auto/xErrorMeters", sample.x - pose.getX());
		Logger.recordOutput("/Auto/yErrorMeters", sample.y - pose.getY());
		Logger.recordOutput(
				"/Auto/headingErrorRad", sample.heading - pose.getRotation().getRadians());
		Logger.recordOutput("/Auto/xFFMps", xff);
		Logger.recordOutput("/Auto/yFFMps", yff);
		Logger.recordOutput("/Auto/omegaFFRadPerSec", wff);
		Logger.recordOutput("/Auto/xFBMps", xfb);
		Logger.recordOutput("/Auto/yFBMps", yfb);
		Logger.recordOutput("/Auto/omegaFBRadPerSec", wfb);
		Logger.recordOutput("/Auto/commandedVxMps", xff + xfb);
		Logger.recordOutput("/Auto/commandedVyMps", yff + yfb);
		Logger.recordOutput("/Auto/commandedOmegaRadPerSec", wff + wfb);
		Logger.recordOutput("/Auto/trajectoryTimestampSec", sample.t);

		drive.runVelocity(speeds);
	}
}
