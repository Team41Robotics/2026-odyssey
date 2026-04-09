package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.IntakeHW;

@SuppressWarnings("static-access")
public class ExtendOut extends Command {
	public ExtendOut() {
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.targetPivotVoltage = IntakeHW.PIVOT_VOLTAGE;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetPivotVoltage = 0;
		intake.targetIntakeVoltage = IntakeHW.INTAKE_VOLTAGE;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
