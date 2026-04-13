package frc.robot.subsystem.indexer;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IndexerInputs {
	public double rollerVoltageVolts;
	public double rollerCurrentAmps;
	public double rollerVelocityRPM = 0.0;
	public double rollerBusVoltageVolts = 0.0;
	public double rollerBusCurrentAmps = 0.0;

	public double indexerVoltageVolts;
	public double indexerCurrentAmps;
	public double indexerVelocityRPM = 0.0;
	public double indexerBusVoltageVolts = 0.0;
	public double indexerBusCurrentAmps = 0.0;
}
