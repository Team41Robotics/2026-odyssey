package frc.robot.subsystem.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
	public IntakeHW hw = new IntakeHW();
	public IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	// Per-loop actuator targets written by commands before actuate()
	public double targetExtendVoltage = 0;
	public double targetIntakeVoltage = 0;

	public void init() {
		hw.init();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Intake", inputs);

		Logger.recordOutput("/Intake/targetIntakeVoltage", targetIntakeVoltage);
	}

	public void actuate() {
		hw.actuate(inputs, targetExtendVoltage, targetIntakeVoltage);
	}
}
