package frc.robot.subsystem.indexer;

import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem {
	public IndexerHW hw = new IndexerHW();
	public IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

	public double targetRollerVoltage = 0;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Indexer", inputs);

		Logger.recordOutput("/Indexer/targetRollerVoltage", targetRollerVoltage);
	}

	public void actuate() {
		hw.actuate(inputs, targetRollerVoltage);
	}

	public void setRollerVoltage(double voltage) {
		targetRollerVoltage = voltage;
	}
}
