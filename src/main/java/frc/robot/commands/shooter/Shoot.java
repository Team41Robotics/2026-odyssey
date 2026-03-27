package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("static-access")
public class Shoot extends Command {
	public static final double FEEDER_VOLTAGE = 9.6; // FIXME.

	public Shoot() {
		addRequirements(shooter, indexer);
	}

	@Override
	public void execute() {
		double dist = Targetting.targetRelative(Targetting.shootOnTheFly(Targetting.hubTarget())).getNorm();
		Targetting.ShotParameters params = Targetting.shotSpeeds(dist);

		shooter.targetFlywheelRPM = params.flywheelRPM();
		indexer.targetIndexerVoltage = shooter.onTarget ? FEEDER_VOLTAGE : 0.0;
		indexer.targetRollerVoltage = shooter.onTarget ? 3 : 0;

		Logger.recordOutput("/Shoot/dist", dist);
		Logger.recordOutput("/Shoot/targetRPM", params.flywheelRPM());
		Logger.recordOutput("/Shoot/onTarget", shooter.onTarget);
	}

	@Override
	public void end(boolean interrupted) {
		indexer.targetIndexerVoltage = 0;
		indexer.targetRollerVoltage = 0;
	}
}
