package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("static-access")
public class Shoot extends Command {
	public static final double FEEDER_VOLTAGE = 12;

	public Shoot() {
		addRequirements(indexer);
	}

	@Override
	public void execute() {
		double dist = Targetting.targetRelative(Targetting.shootOnTheFly(Targetting.hubTarget()))
				.getNorm();
		Targetting.ShotParameters params = Targetting.shotSpeeds(dist);

		Logger.recordOutput("/Shoot/distMeters", dist);
		Logger.recordOutput("/Shoot/targetRPM", params.flywheelRPM());
		Logger.recordOutput("/Shoot/onTarget", shooter.onTarget);

		if (!shooter.onTarget) {
			indexer.targetIndexerVoltage = 0.0;
			indexer.targetRollerVoltage = 0.0;
			return;
		}

		double now = Timer.getTimestamp();
		indexer.targetIndexerVoltage = FEEDER_VOLTAGE;
		indexer.targetRollerVoltage = FEEDER_VOLTAGE * cbrt(cbrt((sin(2 * PI * now/0.7) + 0.77) / 1.6));
	}

	@Override
	public void end(boolean interrupted) {
		indexer.targetIndexerVoltage = 0;
		indexer.targetRollerVoltage = 0;
	}
}
