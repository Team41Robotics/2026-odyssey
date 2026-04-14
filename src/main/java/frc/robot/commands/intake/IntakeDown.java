package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDown extends Command {
	public static final double HIGH_VOLTAGE = 12.0;

	public double voltage;

	public IntakeDown() {
		this(HIGH_VOLTAGE);
	}

	public IntakeDown(double voltage) {
		this.voltage = voltage;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.targetPivotPositionRadians = 0.0;
		intake.targetPivotFeedforwardBiasVolts = -intake.intakeDownBiasVolts.get();
		intake.targetIntakeVoltage = voltage;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetPivotPositionRadians = null;
		intake.targetPivotVoltage = 0;
		intake.targetPivotFeedforwardBiasVolts = 0;
		intake.targetIntakeVoltage = 0;
	}
}
