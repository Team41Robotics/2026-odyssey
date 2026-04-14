package frc.robot.subsystem.indexer;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IndexerInputs {
	public double rollerVoltageVolts;
	public double rollerCurrentAmps;
	public double rollerVelocityRPM;
	public double rollerBusVoltageVolts;
	public double rollerBusCurrentAmps;

	public double indexerVoltageVolts;
	public double indexerCurrentAmps;
	public double indexerVelocityRPM;
	public double indexerBusVoltageVolts;
	public double indexerBusCurrentAmps;

	public double rollerTsSec;
	public double indexerTsSec;
}
