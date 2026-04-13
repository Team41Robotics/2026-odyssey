package frc.robot.commands.indexer;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

@SuppressWarnings("static-access")
public class RunIndexer extends Command {
	public double rollerVoltage;
	public double indexerVoltage;

	public RunIndexer(double rollerVoltage, double indexerVoltage) {
		this.rollerVoltage = rollerVoltage;
		this.indexerVoltage = indexerVoltage;
		addRequirements(indexer);
	}

	@Override
	public void execute() {
		indexer.targetRollerVoltage = rollerVoltage;
		indexer.targetIndexerVoltage = indexerVoltage;
	}

	@Override
	public void end(boolean interrupted) {
		indexer.targetRollerVoltage = 0;
		indexer.targetIndexerVoltage = 0;
	}
}
