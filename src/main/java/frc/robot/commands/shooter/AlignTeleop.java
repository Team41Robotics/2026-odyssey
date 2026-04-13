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
public class AlignTeleop extends Command {
	public static final double AIM_kP = Align.AIM_kP;
	public static final double AIM_kI = Align.AIM_kI;
	public static final double AIM_kD = Align.AIM_kD;

	public final PIDController headingController = new PIDController(AIM_kP, AIM_kI, AIM_kD);

	public AlignTeleop() {
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		headingController.enableContinuousInput(-PI, PI);
		headingController.reset();
	}

	@Override
	public void execute() {
		ShootTeleop.TeleopTarget teleopTarget = ShootTeleop.getTeleopTarget();
		Translation2d target = teleopTarget.target();
		Translation2d virtualTarget = Targetting.shootOnTheFly(target);

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

		Logger.recordOutput("/AlignTeleop/targetState", teleopTarget.state());
		Logger.recordOutput("/AlignTeleop/joystickTarget", new Pose2d(target, Rotation2d.kZero));
		Logger.recordOutput("/AlignTeleop/virtualTarget", new Pose2d(virtualTarget, Rotation2d.kZero));
		Logger.recordOutput("/AlignTeleop/targetAngle", targetAngle);
		Logger.recordOutput("/AlignTeleop/headingError", headingController.getError());
	}
}
