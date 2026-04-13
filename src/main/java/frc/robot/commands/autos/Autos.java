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
import org.littletonrobotics.junction.Logger;

public class Autos {
	// FIXME: tune trajectory tracking PID gains
	public static PIDController xController = new PIDController(10.0, 0, 0);
	public static PIDController yController = new PIDController(10.0, 0, 0);
	public static PIDController thetaController = new PIDController(7, 0, 0);

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

		Logger.recordOutput("/Auto/targetPose", sample.getPose());
		Logger.recordOutput("/Auto/targetVx", sample.vx);
		Logger.recordOutput("/Auto/targetVy", sample.vy);
		Logger.recordOutput("/Auto/targetOmega", sample.omega);
		Logger.recordOutput("/Auto/xError", sample.x - pose.getX());
		Logger.recordOutput("/Auto/yError", sample.y - pose.getY());

		drive.runVelocity(speeds);
	}
}
