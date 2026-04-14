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
			Logger.recordOutput("/Auto/Test/trajectory", traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active().onTrue(traj.cmd());
			traj.done().onTrue(traj.cmd());
			return routine;
		});
		autoChooser.addRoutine("Trench1", () -> {
			var routine = autoFactory.newRoutine("Trench1");
			var traj = ChoreoTraj.Trench1.asAutoTraj(routine);
			Logger.recordOutput("/Auto/Trench1/trajectory", traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active().onTrue(Commands.sequence(
					Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "started")),
					Commands.parallel(traj.cmd(), new IntakeUp())));
			traj.atTime("DeployIntake").onTrue(Commands.sequence(
					Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "deployIntake")),
					new IntakeDown()));
			traj.done().onTrue(Commands.sequence(
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
		Logger.recordOutput("/Auto/targetVx", sample.vx);
		Logger.recordOutput("/Auto/targetVy", sample.vy);
		Logger.recordOutput("/Auto/targetOmega", sample.omega);
		Logger.recordOutput("/Auto/xError", sample.x - pose.getX());
		Logger.recordOutput("/Auto/yError", sample.y - pose.getY());
		Logger.recordOutput("/Auto/headingError", sample.heading - pose.getRotation().getRadians());
		Logger.recordOutput("/Auto/xFF", xff);
		Logger.recordOutput("/Auto/yFF", yff);
		Logger.recordOutput("/Auto/omegaFF", wff);
		Logger.recordOutput("/Auto/xFB", xfb);
		Logger.recordOutput("/Auto/yFB", yfb);
		Logger.recordOutput("/Auto/omegaFB", wfb);
		Logger.recordOutput("/Auto/commandedVx", xff + xfb);
		Logger.recordOutput("/Auto/commandedVy", yff + yfb);
		Logger.recordOutput("/Auto/commandedOmega", wff + wfb);
		Logger.recordOutput("/Auto/trajectoryTimestamp", sample.t);

		drive.runVelocity(speeds);
	}
}
