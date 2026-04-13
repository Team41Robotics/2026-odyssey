package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeUp extends Command {
	public static final double PIVOT_UP_VOLTAGE = 1.0;
	public static final double PIVOT_kG = 0.9; // TODO eyeballed
	public static final double PIVOT_UP_POS = 75.0 / 180 * PI;
	public static final double INTAKE_UP_VOLTAGE = 5.0;

	public IntakeUp() {
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.targetPivotVoltage = intake.inputs.pivotPosRadians < PIVOT_UP_POS ? PIVOT_UP_VOLTAGE : 0;
		intake.targetPivotVoltage += PIVOT_kG * cos(intake.inputs.pivotPosRadians);
		intake.targetIntakeVoltage = INTAKE_UP_VOLTAGE;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetPivotVoltage = 0;
		intake.targetIntakeVoltage = 0;
	}
}
