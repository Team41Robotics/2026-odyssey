package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.Util;

public class Targetting {
	public record ShotParameters(double flywheelRPM, double timeOfFlight, double distance) {}

	// {dist (m), offset (m)} — offset compensates for overshoot at that distance
	public static final double[][] OFFSET_TABLE = {
		{1.5, 0.0},
		{6.0, 0.0},
	};

	// {dist (m), rpm, tof (s)} — FIXME. tune all values
	public static final double[][] SHOT_TABLE = {
		{1.5, 2100, 0.90}, //100
		{2.0, 2200, 1.08}, //100
		{2.5, 2300, 1.23}, //150
		{3.0, 2450, 1.28}, //150
		{3.5, 2650, 1.34}, //200
		{4.0, 2850, 1.36}, //200
		{4.5, 3100, 1.40}, //250
		{5.0, 3350, 1.43}, //250
		{5.5, 3650, 1.46}, //300
		{6.0, 3950, 1.48}, //300
		{6.5, 4300, 1.51}, //350
		{7.0, 4650, 1.53}, //350
		{7.5, 5050, 1.57}, //400
		{8.0, 5450, 1.61}, //400
		{8.5, 5900, 1.66}, //450
		{9.0, 6450, 1.70}, //450
		{8.5, 6950, 1.75}, //500
		{9.0, 7450, 1.81}, //500
		{9.5, 8000, 1.87}, //550
		{10.0, 8550, 1.94}, //550
		
	};

	public static double lerpOffset(double distance) {
		if (distance <= OFFSET_TABLE[0][0]) return OFFSET_TABLE[0][1];
		if (distance >= OFFSET_TABLE[OFFSET_TABLE.length - 1][0]) return OFFSET_TABLE[OFFSET_TABLE.length - 1][1];
		for (int i = 0; i < OFFSET_TABLE.length - 1; i++) {
			double[] lo = OFFSET_TABLE[i], hi = OFFSET_TABLE[i + 1];
			if (distance >= lo[0] && distance <= hi[0]) {
				double t = (distance - lo[0]) / (hi[0] - lo[0]);
				return lo[1] + t * (hi[1] - lo[1]);
			}
		}
		return 0.0;
	}

	public static ShotParameters shotSpeeds(double distance) {
		distance -= lerpOffset(distance);
		if (distance <= SHOT_TABLE[0][0]) {
			double[] lo = SHOT_TABLE[0], hi = SHOT_TABLE[1];
			double t = (distance - lo[0]) / (hi[0] - lo[0]);
			return new ShotParameters(lo[1] + t * (hi[1] - lo[1]), lo[2] + t * (hi[2] - lo[2]), distance);
		}
		if (distance >= SHOT_TABLE[SHOT_TABLE.length - 1][0]) {
			double[] lo = SHOT_TABLE[SHOT_TABLE.length - 2], hi = SHOT_TABLE[SHOT_TABLE.length - 1];
			double t = (distance - lo[0]) / (hi[0] - lo[0]);
			return new ShotParameters(lo[1] + t * (hi[1] - lo[1]), lo[2] + t * (hi[2] - lo[2]), distance);
		}
		for (int i = 0; i < SHOT_TABLE.length - 1; i++) {
			double[] lo = SHOT_TABLE[i], hi = SHOT_TABLE[i + 1];
			if (distance >= lo[0] && distance <= hi[0]) {
				double t = (distance - lo[0]) / (hi[0] - lo[0]);
				return new ShotParameters(lo[1] + t * (hi[1] - lo[1]), lo[2] + t * (hi[2] - lo[2]), distance);
			}
		}
		return new ShotParameters(0, 0, distance);
	}

	/** Hub target, flipped for alliance. */
	public static Translation2d hubTarget() {
		return Util.flipIfRed(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
	}

	/** Vector from robot pose to target (field-relative). */
	public static Translation2d targetRelative(Translation2d target) {
		return target.minus(drive.getPose().getTranslation());
	}

	/** Field-relative heading angle to target (radians). */
	public static double shotAngle(Translation2d target) {
		return targetRelative(target).getAngle().getRadians();
	}

	/**
	 * Shift target back by robot velocity × time-of-flight to compensate for robot motion.
	 * CTRE state speeds are robot-relative; rotate to field frame first.
	 */
	public static Translation2d targetOnTheFly(Translation2d target, double timeOfFlight) {
		ChassisSpeeds speeds = drive.getChassisSpeeds();
		Rotation2d heading = drive.getRotation();
		Translation2d vel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(heading);
		return target.minus(vel.times(timeOfFlight));
	}

	/** Iterative convergence (5 passes) for virtual target compensated for robot motion. */
	public static Translation2d shootOnTheFly(Translation2d target) {
		Translation2d newTarget = target;
		for (int i = 0; i < 5; i++) {
			ShotParameters params = shotSpeeds(targetRelative(newTarget).getNorm());
			newTarget = targetOnTheFly(target, params.timeOfFlight());
		}
		return newTarget;
	}
}
