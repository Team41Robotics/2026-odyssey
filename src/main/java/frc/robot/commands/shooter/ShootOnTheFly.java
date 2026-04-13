package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Util;
import frc.robot.commands.shooter.Targetting.ShotParameters;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheFly extends Command {
	public Translation2d origTarget, target;

	public ShootOnTheFly() {
		this(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
	}

	public ShootOnTheFly(Translation2d target) {
		this.origTarget = target;
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		target = Util.flipIfRed(origTarget);
	}

	@Override
	public void execute() {
		Translation2d virtualTarget = Targetting.shootOnTheFly(target);

		double distance = Targetting.targetRelative(virtualTarget).getNorm();
		ShotParameters params = Targetting.shotSpeeds(distance);
		shooter.targetFlywheelRPM = params.flywheelRPM();

		Logger.recordOutput("/Targetting/targetPose", virtualTarget);
		Logger.recordOutput("/Targetting/distance", distance);
		Logger.recordOutput("/Targetting/timeOfFlight", params.timeOfFlight());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
