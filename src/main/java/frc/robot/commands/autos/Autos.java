package frc.robot.commands.autos;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.commands.intake.ExtendOut;
import frc.robot.commands.shooter.Align;
import frc.robot.commands.shooter.Shoot;
import org.littletonrobotics.junction.Logger;

public class Autos {
	// FIXME: tune trajectory tracking PID gains
	public static PIDController xController = new PIDController(10.0, 0, 0);
	public static PIDController yController = new PIDController(10.0, 0, 0);
	public static PIDController thetaController = new PIDController(10, 0, 0);

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

	public static AutoRoutine trenchAuto() {
		AutoRoutine routine = autoFactory.newRoutine("TrenchAuto");
		AutoTrajectory traj = ChoreoTraj.TrenchAuto.asAutoTraj(routine);

		routine.active()
				.onTrue(Commands.sequence(
						new ExtendOut().withTimeout(0.5),
						new Align().withTimeout(3.0),
						new Shoot().withTimeout(6.0),
						traj.cmd(),
						new Align().withTimeout(3.0),
						new Shoot().withTimeout(6.0)));

		return routine;
	}
}
