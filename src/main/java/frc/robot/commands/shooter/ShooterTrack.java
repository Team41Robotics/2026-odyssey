package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Default command for the shooter: always tracks the hub and spins the flywheel to the
 * correct speed for the current distance. Does not aim the drivetrain.
 */
@SuppressWarnings("static-access")
public class ShooterTrack extends Command {
	public ShooterTrack() {
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		Translation2d target = Targetting.hubTarget();
		double dist = Targetting.targetRelative(target).getNorm();
		Targetting.ShotParameters params = Targetting.shotSpeeds(dist);

		shooter.targetFlywheelRPM = params.flywheelRPM();

		Logger.recordOutput("/ShooterTrack/distMeters", dist);
		Logger.recordOutput("/ShooterTrack/targetRPM", params.flywheelRPM());
	}

	@Override
	public boolean runsWhenDisabled() {
		return false;
	}
}
