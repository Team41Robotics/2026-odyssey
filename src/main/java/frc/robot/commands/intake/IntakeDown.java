package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDown extends Command {
	public static final double PIVOT_kP = 4;
	public static final double PIVOT_kG = 0.9; // TODO eyeballed
	public static final double PIVOT_kD = 0.0;
	public static final double HIGH_VOLTAGE = 12.0;
	public static final double CURRENT_THRESHOLD = 100.0;
	public static final double REVERSE_DURATION_SEC = 0.2;

	public double voltage;
	public boolean reversing = false;
	public double reverseStartSec = 0;

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
				+ PIVOT_kG * cos(intake.inputs.pivotPosRadians);

		double now = Timer.getTimestamp();
		if (reversing) {
			intake.targetIntakeVoltage = -HIGH_VOLTAGE;
			if (now - reverseStartSec >= REVERSE_DURATION_SEC) {
				reversing = false;
			}
		} else {
			intake.targetIntakeVoltage = voltage;
			if (intake.inputs.intakeCurrentAmps >= CURRENT_THRESHOLD) {
				reversing = true;
				reverseStartSec = now;
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetPivotVoltage = 0;
		intake.targetIntakeVoltage = 0;
		reversing = false;
	}
}
