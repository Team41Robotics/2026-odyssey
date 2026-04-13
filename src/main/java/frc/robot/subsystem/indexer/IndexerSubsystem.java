package frc.robot.subsystem.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
	public IndexerHW hw = new IndexerHW();
	public IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

	public double targetRollerVoltage = 0;
	public double targetIndexerVoltage = 0;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Indexer", inputs);
	}

	public void actuate() {
		Logger.recordOutput("/Indexer/targetRollerVoltage", targetRollerVoltage);
		Logger.recordOutput("/Indexer/targetIndexerVoltage", targetIndexerVoltage);
		hw.actuate(inputs, targetRollerVoltage, targetIndexerVoltage);
	}
}
