package frc.robot.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ShooterInputs {
	public double flywheelVelocityRPM;
	public double flywheelVoltageVolts;
	public double flywheelCurrentAmps;
	public double flywheelBusVoltageVolts;
	public double flywheelBusCurrentAmps;
	public double flywheelTsSec;

	public double elevatorPosRotations;
	public double elevatorVelRotationsPerSec;
	public double elevatorVoltageVolts;
	public double elevatorCurrentAmps;
	public double elevatorBusVoltageVolts;
	public double elevatorBusCurrentAmps;
	public double elevatorTsSec;
}
