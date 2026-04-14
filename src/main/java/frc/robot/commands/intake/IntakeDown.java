package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDown extends Command {
	public static final double PIVOT_kP = 4;
	public static final double PIVOT_kG = 0.9; // TODO eyeballed
	public static final double PIVOT_kD = 0.3;
	public static final double PIVOT_DOWN_V = 2;
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
		intake.targetPivotVoltage = -PIVOT_kP * intake.inputs.pivotPosRadians
				- PIVOT_kD * intake.inputs.pivotVelRadiansPerSec
				+ PIVOT_kG * cos(intake.inputs.pivotPosRadians) - PIVOT_DOWN_V;

		intake.targetIntakeVoltage = voltage;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetPivotVoltage = 0;
		intake.targetIntakeVoltage = 0;
	}
}
