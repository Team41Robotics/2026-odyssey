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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LoggedAutoChooser;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeZero;
import frc.robot.commands.shooter.AlignTeleop;
import frc.robot.commands.shooter.Shoot;
import org.littletonrobotics.junction.Logger;

public class Autos {
	public static PIDController xController = new PIDController(7.0, 0, 0);
	public static PIDController yController = new PIDController(7.0, 0, 0);
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
		autoChooser.addRoutine("Trench1", () -> {
			var routine = autoFactory.newRoutine("Trench1");
			var traj = ChoreoTraj.Trench1.asAutoTraj(routine);
			Logger.recordOutput(
					"/Auto/Trench1/trajectory",
					traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active()
					.onTrue(Commands.sequence(Commands.runOnce(
							() -> Logger.recordOutput("/Auto/Trench1/phase", "started")),
							traj.cmd()));
			traj.atTime("DeployIntake")
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "deployIntake")),
							new IntakeZero().andThen(new IntakeDown())));
			traj.done()
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "aligning")),
							new AlignTeleop().withTimeout(0.5),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "firing")),
							new Shoot().withTimeout(3.0),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Trench1/phase", "done"))));
			return routine;
		});
		autoChooser.addRoutine("GreedyDoubleDip", () -> {
			var routine = autoFactory.newRoutine("GreedyDoubleDip");
			var traj = ChoreoTraj.greedydoubledip$0.asAutoTraj(routine);
			Logger.recordOutput(
					"/Auto/GreedyDoubleDip/trajectory",
					traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active()
					.onTrue(Commands.sequence(new WaitCommand(0.5), Commands.runOnce(
							() -> Logger.recordOutput("/Auto/GreedyDoubleDip/phase", "started")),
							new IntakeZero().andThen(new IntakeDown()),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/GreedyDoubleDip/phase", "deployIntake")),
							traj.cmd()));
			traj.done()
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/GreedyDoubleDip/phase", "firing")),
							new Shoot().withTimeout(3.0),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/GreedyDoubleDip/phase", "done"))));
			traj = ChoreoTraj.greedydoubledip$1.asAutoTraj(routine);
			Logger.recordOutput(
					"/Auto/GreedyDoubleDip/trajectory",
					traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active()
					.onTrue(Commands.sequence(Commands.runOnce(
							() -> Logger.recordOutput("/Auto/GreedyDoubleDip/phase", "started")),
							traj.cmd()));
			traj.done()
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/GreedyDoubleDip/phase", "firing")),
							new Shoot().withTimeout(3.0),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/GreedyDoubleDip/phase", "done"))));
			return routine;
		});
		autoChooser.addRoutine("Mid", () -> {
			var routine = autoFactory.newRoutine("Mid");
			var traj = ChoreoTraj.Mid.asAutoTraj(routine);
			Logger.recordOutput(
					"/Auto/Mid/trajectory",
					traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active()
					.onTrue(Commands.sequence(Commands.runOnce(
							() -> Logger.recordOutput("/Auto/Mid/phase", "started")),
							traj.cmd()));
			traj.done()
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Mid/phase", "aligning")),
							new AlignTeleop().withTimeout(0.5),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Mid/phase", "firing")),
							new Shoot().withTimeout(3.0),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Mid/phase", "deployIntake")),
							new IntakeZero().andThen(new IntakeDown()),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/Mid/phase", "done"))
						));


		return routine;
		});
		autoChooser.addRoutine("OtherDoubleTrench", () -> {
			var routine = autoFactory.newRoutine("OtherDoubleTrench");
			var traj = ChoreoTraj.OtherDoubleTrench$0.asAutoTraj(routine);
			Logger.recordOutput(
					"/Auto/OtherDoubleTrench/trajectory",
					traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active()
					.onTrue(Commands.sequence(new WaitCommand(0.5), Commands.runOnce(
							() -> Logger.recordOutput("/Auto/OtherDoubleTrench/phase", "started")),
							new IntakeZero().andThen(new IntakeDown()),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/OtherDoubleTrench/phase", "deployIntake")),
							traj.cmd()));
							
			traj.done()
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/OtherDoubleTrench/phase", "firing")),
							new Shoot().withTimeout(3.5),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/OtherDoubleTrench/phase", "done"))));
			traj = ChoreoTraj.OtherDoubleTrench$1.asAutoTraj(routine);
			Logger.recordOutput(
					"/Auto/OtherDoubleTrench/trajectory",
					traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active()
					.onTrue(Commands.sequence(Commands.runOnce(
							() -> Logger.recordOutput("/Auto/OtherDoubleTrench/phase", "started")),
							traj.cmd()));
			traj.done()
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/OtherDoubleTrench/phase", "firing")),
							new Shoot().withTimeout(3.5),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/OtherDoubleTrench/phase", "done"))));
			return routine;
		});
		autoChooser.addRoutine("realtoptrenchdouble", () -> {
			var routine = autoFactory.newRoutine("realtoptrenchdouble");
			var traj = ChoreoTraj.realtoptrenchdouble$0.asAutoTraj(routine);
			Logger.recordOutput(
					"/Auto/realtoptrenchdouble/trajectory",
					traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active()
					.onTrue(Commands.sequence(
							new WaitCommand(0.5),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/realtoptrenchdouble/phase", "deployIntake")),
							new IntakeZero().andThen(new IntakeDown()),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/realtoptrenchdouble/phase", "started")),
							traj.cmd()));
			traj.done()
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/realtoptrenchdouble/phase", "firing")),
							new Shoot().withTimeout(3.0),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/realtoptrenchdouble/phase", "done"))));
			traj = ChoreoTraj.realtoptrenchdouble$1.asAutoTraj(routine);
			Logger.recordOutput(
					"/Auto/realtoptrenchdouble/trajectory",
					traj.<SwerveSample>getRawTrajectory().getPoses());
			routine.active()
					.onTrue(Commands.sequence(Commands.runOnce(
							() -> Logger.recordOutput("/Auto/realtoptrenchdouble/phase", "started")),
							traj.cmd()));
			traj.done()
					.onTrue(Commands.sequence(
							Commands.runOnce(() -> Logger.recordOutput("/Auto/realtoptrenchdouble/phase", "firing")),
							new Shoot().withTimeout(3.0),
							Commands.runOnce(() -> Logger.recordOutput("/Auto/realtoptrenchdouble/phase", "done"))));
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
