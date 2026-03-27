package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;
import frc.robot.commands.drive.FieldOrientedDrive;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("static-access")
public class Shoot extends Command {
	public static final double AIM_kP = 5.0;
	public static final double AIM_kI = 0.0;
	public static final double AIM_kD = 0.0;
	public static final double FEEDER_VOLTAGE = 9.6; // FIXME.

	public final PIDController headingController = new PIDController(AIM_kP, AIM_kI, AIM_kD);

	public Shoot() {
		addRequirements(shooter, drive);
	}

	@Override
	public void initialize() {
		headingController.enableContinuousInput(-PI, PI);
		headingController.reset();
	}

	@Override
	public void execute() {
		Translation2d virtualTarget = Targetting.shootOnTheFly(Targetting.hubTarget());
		double dist = Targetting.targetRelative(virtualTarget).getNorm();
		Targetting.ShotParameters params = Targetting.shotSpeeds(dist);

		shooter.targetFlywheelRPM = params.flywheelRPM();

		double mag = hypot(controls.leftY(), controls.leftX());
		double v = Util.squareCurve(Util.deadband(mag, FieldOrientedDrive.DEADBAND));
		double theta = atan2(controls.leftX(), controls.leftY());

		double targetAngle = Targetting.shotAngle(virtualTarget);
		double omega = headingController.calculate(drive.getRotation().getRadians(), targetAngle);

		drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
				v * cos(theta) * FieldOrientedDrive.MAX_SPEED * FieldOrientedDrive.SPEED_MUL,
				v * sin(theta) * FieldOrientedDrive.MAX_SPEED * FieldOrientedDrive.SPEED_MUL,
				omega,
				drive.getRotation()));

		indexer.setRollerVoltage(shooter.onTarget ? FEEDER_VOLTAGE : 0.0);

		Logger.recordOutput(
				"/Shoot/virtualTarget", new edu.wpi.first.math.geometry.Pose2d(virtualTarget, Rotation2d.kZero));
		Logger.recordOutput("/Shoot/dist", dist);
		Logger.recordOutput("/Shoot/targetRPM", params.flywheelRPM());
		Logger.recordOutput("/Shoot/targetAngle", targetAngle);
		Logger.recordOutput("/Shoot/headingError", headingController.getError());
		Logger.recordOutput("/Shoot/onTarget", shooter.onTarget);
	}

	@Override
	public void end(boolean interrupted) {
		indexer.setRollerVoltage(0);
	}
}
