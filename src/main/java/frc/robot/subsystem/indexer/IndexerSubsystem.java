package frc.robot.subsystem.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
	public IndexerHW hw = new IndexerHW();
	public IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

	public double targetIndexerVoltage = 0;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Indexer", inputs);

		Logger.recordOutput("/Indexer/targetIndexerVoltage", targetIndexerVoltage);
	}

	public void actuate() {
		hw.actuate(inputs, targetIndexerVoltage);
	}

	public void setIndexerVoltage(double voltage) {
		targetIndexerVoltage = voltage;
	}
}
