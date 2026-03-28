package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.IntakeHW;

@SuppressWarnings("static-access")
public class ExtendIn extends Command {
	public ExtendIn() {
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.targetExtendVoltage = IntakeHW.EXTEND_IN_VOLTAGE;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetExtendVoltage = 0;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
