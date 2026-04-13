package frc.robot.subsystem.intake;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IntakeInputs {
	public double pivotPosRadians;
	public double pivotVelRadiansPerSec;
	public double pivotVoltageVolts;
	public double pivotCurrentAmps;
	public double pivotBusVoltageVolts;
	public double pivotBusCurrentAmps;

	public double intakeVoltageVolts;
	public double intakeCurrentAmps;

	public double rollerVelocityRPS = 0.0;
	public double rollerBusVoltageVolts = 0.0;
	public double rollerBusCurrentAmps = 0.0;
	public double pivotVelocityRPS = 0.0;
}
