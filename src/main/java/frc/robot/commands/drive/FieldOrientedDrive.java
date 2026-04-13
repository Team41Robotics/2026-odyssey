package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;
import static frc.robot.Util.*;
import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

@SuppressWarnings("static-access")
public class FieldOrientedDrive extends Command {
	public static final double MAX_SPEED = drive.getMaxLinearSpeedMetersPerSec();
	public static final double MAX_ANGULAR_RATE = drive.getMaxAngularSpeedRadPerSec();
	public static final double DEADBAND = 0.10;
	public static final double TURN_DEADBAND = 0.10;
	public static final double SPEED_MUL = 1.0;
	public static final double W_MUL = 1.0;

	public FieldOrientedDrive() {
		addRequirements(drive);
	}

	@Override
	public void execute() {
		double vx = controls.leftY();
		double vy = controls.leftX();
		double w = controls.rightX();

		double mag = hypot(vx, vy);
		double v = squareCurve(deadband(mag, DEADBAND));
		double wc = squareCurve(deadband(w, TURN_DEADBAND));
		double theta = atan2(vy, vx);

		drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
				v * cos(theta) * MAX_SPEED * SPEED_MUL,
				v * sin(theta) * MAX_SPEED * SPEED_MUL,
				wc * MAX_ANGULAR_RATE * W_MUL,
				drive.getRotation().rotateBy(isRed() ? Rotation2d.k180deg : Rotation2d.kZero)));
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}
}
