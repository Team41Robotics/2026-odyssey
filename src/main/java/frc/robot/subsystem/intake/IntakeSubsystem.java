package frc.robot.subsystem.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
	public IntakeHW hw = new IntakeHW();
	public IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	public double targetPivotVoltage = 0;
	public double targetIntakeVoltage = 0;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Intake", inputs);

		if (robot.isDisabled()) {
			targetPivotVoltage = 0;
		}
	}

	public void actuate() {
		Logger.recordOutput("/Intake/targetPivotVoltage", targetPivotVoltage);
		Logger.recordOutput("/Intake/targetIntakeVoltage", targetIntakeVoltage);

		hw.actuate(inputs, targetPivotVoltage, targetIntakeVoltage);
	}
}
