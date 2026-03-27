package frc.robot.commands.autos;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

public class Autos {
	// FIXME: tune trajectory tracking PID gains
	public static PIDController xController = new PIDController(5.0, 0, 0);
	public static PIDController yController = new PIDController(5.0, 0, 0);
	public static PIDController thetaController = new PIDController(3.0, 0, 0);

	public static void init() {
		thetaController.enableContinuousInput(-PI, PI);
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
