package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeUp extends Command {
	public static final double INTAKE_UP_VOLTAGE = 5.0;

	public IntakeUp() {
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.targetPivotPositionRadians = Math.toRadians(intake.intakeUpDeg.get());
		intake.targetIntakeVoltage = INTAKE_UP_VOLTAGE;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetPivotPositionRadians = null;
		intake.targetPivotVoltage = 0;
		intake.targetIntakeVoltage = 0;
	}
}
