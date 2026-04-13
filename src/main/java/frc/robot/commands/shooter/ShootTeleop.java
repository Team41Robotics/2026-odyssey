package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Util;
import frc.robot.commands.shooter.Targetting.ShotParameters;
import org.littletonrobotics.junction.Logger;

public class ShootTeleop extends Command {
	public static final double JOYSTICK_SCALE = 0.3; // meters per full joystick deflection
	public static final double TRENCH_Y_OFFSET = 1.7; // meters offset when shooting over trench

	public record TeleopTarget(Translation2d target, String state) {}

	public ShootTeleop() {
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		TeleopTarget teleopTarget = getTeleopTarget();
		Translation2d target = teleopTarget.target();
		String state = teleopTarget.state();

		Translation2d virtualTarget = Targetting.shootOnTheFly(target);

		double distance = Targetting.targetRelative(virtualTarget).getNorm();
		ShotParameters params = Targetting.shotSpeeds(distance);
		shooter.targetFlywheelRPM = params.flywheelRPM();

		Logger.recordOutput("/Targetting/state", state);
		Logger.recordOutput("/Targetting/targetPose", new Pose2d(virtualTarget, new Rotation2d()));
		Logger.recordOutput("/Targetting/joystickTarget", new Pose2d(target, new Rotation2d()));
		Logger.recordOutput("/Targetting/distance", distance);
		Logger.recordOutput("/Targetting/timeOfFlight", params.timeOfFlight());

		// Zone boundary lines
		double xTrenchMin = FieldConstants.LinesVertical.hubCenter - FieldConstants.LeftBump.width / 2.0;
		double xTrenchMax = FieldConstants.LinesVertical.hubCenter + FieldConstants.LeftBump.width / 2.0;
		double fieldW = FieldConstants.fieldWidth;
		Pose2d[] trenchMinLine = new Pose2d[] {
			new Pose2d(xTrenchMin, 0, new Rotation2d()), new Pose2d(xTrenchMin, fieldW, new Rotation2d())
		};
		Pose2d[] trenchMaxLine = new Pose2d[] {
			new Pose2d(xTrenchMax, 0, new Rotation2d()), new Pose2d(xTrenchMax, fieldW, new Rotation2d())
		};
		Logger.recordOutput("/Targetting/trenchMinLine", trenchMinLine);
		Logger.recordOutput("/Targetting/trenchMaxLine", trenchMaxLine);
	}

	public static TeleopTarget getTeleopTarget() {
		Translation2d hubCenter = FieldConstants.Hub.innerCenterPoint.toTranslation2d();

		// Convert robot position to alliance-relative coordinates (blue perspective)
		Translation2d robotPos = Util.flipIfRed(drive.getPose().getTranslation());
		double robotX = robotPos.getX();
		double robotY = robotPos.getY();
		double xTrenchMin = FieldConstants.LinesVertical.hubCenter - FieldConstants.LeftBump.width / 2.0;
		double xTrenchMax = FieldConstants.LinesVertical.hubCenter + FieldConstants.LeftBump.width / 2.0;
		double centerY = FieldConstants.fieldWidth / 2.0;

		Translation2d target;
		String state;

		if (robotX < xTrenchMin) {
			// Before trench - shoot at hub with joystick offset
			Translation2d offset =
					new Translation2d(controls.thirdY() * JOYSTICK_SCALE, controls.thirdX() * JOYSTICK_SCALE);
			target = Util.flipIfRed(hubCenter.plus(offset));
			state = "SHOOT_HUB";
		} else if (robotX < xTrenchMax) {
			// In trench zone - aim at hub directly
			target = Util.flipIfRed(hubCenter);
			state = "TRENCH";
		} else {
			// Past trench - pass to own side, not through middle
			double yOffset = (robotY < centerY) ? -TRENCH_Y_OFFSET : TRENCH_Y_OFFSET;
			target = Util.flipIfRed(hubCenter.plus(new Translation2d(0, yOffset)));
			state = "PASS";
		}

		return new TeleopTarget(target, state);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
