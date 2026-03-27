package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.drive.TunerConstants;

@SuppressWarnings("static-access")
public class FieldOrientedDrive extends Command {
	public static final double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // FIXME.
	public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // FIXME.
	public static final double DEADBAND = 0.10; // FIXME.
	public static final double TURN_DEADBAND = 0.10; // FIXME.
	public static final double SPEED_MUL = 1.0; // FIXME.
	public static final double W_MUL = 1.0; // FIXME.

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
				drive.getRotation()));
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}

	private static double deadband(double x, double db) {
		if (abs(x) < db) return 0;
		return copySign(((abs(x) - db) / (1 - db)), x);
	}

	private static double squareCurve(double x) {
		return copySign(x * x, x);
	}
}
