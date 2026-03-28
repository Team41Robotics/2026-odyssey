package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;
import frc.robot.commands.drive.FieldOrientedDrive;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("static-access")
public class Align extends Command {
	public static final double AIM_kP = 10.0;
	public static final double AIM_kI = 0.0;
	public static final double AIM_kD = 0.0;

	public final PIDController headingController = new PIDController(AIM_kP, AIM_kI, AIM_kD);

	public Align() {
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		headingController.enableContinuousInput(-PI, PI);
		headingController.reset();
	}

	@Override
	public void execute() {
		Translation2d virtualTarget = Targetting.shootOnTheFly(Targetting.hubTarget());

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

		Logger.recordOutput("/Align/virtualTarget", new Pose2d(virtualTarget, Rotation2d.kZero));
		Logger.recordOutput("/Align/targetAngle", targetAngle);
		Logger.recordOutput("/Align/headingError", headingController.getError());
	}
}
