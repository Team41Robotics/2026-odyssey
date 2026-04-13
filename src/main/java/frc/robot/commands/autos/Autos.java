package frc.robot.commands.autos;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LoggedAutoChooser;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.shooter.Align;
import frc.robot.commands.shooter.Shoot;
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
		autoChooser.addRoutine("Bad", Autos::trenchAuto);
		autoChooser.addRoutine("ShootAuto", Autos::shootAuto);
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

	public static AutoRoutine trenchAuto() {
		AutoRoutine routine = autoFactory.newRoutine("Bad");
		// AutoTrajectory traj = ChoreoTraj.TrenchAuto.asAutoTraj(routine);
		AutoTrajectory traj2 = ChoreoTraj.Bad.asAutoTraj(routine);

		routine.active()
				.onTrue(Commands.sequence(
						Commands.runOnce(() -> Logger.recordOutput("Auto/activeRoutine", "trenchAuto")),
						new Align().withTimeout(1.5),
						new Shoot().withTimeout(3),
						Commands.deadline(traj2.cmd(), new WaitCommand(2).andThen(new IntakeDown())),
						new Align().withTimeout(1.5),
						new Shoot().withTimeout(3)));

		// routine.active()
		// 		.onTrue(//Commands.parallel(
		// 				Commands.sequence(
		// 					traj.cmd(),traj2.cmd()));
		// 						// new Align().withTimeout(3.0),
		// 						// new Shoot().withTimeout(6.0),
		// 						// traj.cmd()
		// 						//new Align().withTimeout(3.0),
		// 						//new Shoot().withTimeout(6.0)));
		// 				//Commands.sequence(new WaitCommand(10), new IntakeDown().withTimeout(3))));

		return routine;
	}

	public static AutoRoutine shootAuto() {
		AutoRoutine rountine = autoFactory.newRoutine("ShootAuto");
		rountine.active()
				.onTrue(Commands.sequence(
						Commands.runOnce(() -> Logger.recordOutput("Auto/activeRoutine", "shootAuto")),
						new Align().withTimeout(3),
						new Shoot().withTimeout(6)));
		return rountine;
	}
}
