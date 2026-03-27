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
	public void initialize() {
		intake.extendTarget = IntakeHW.EXTEND_IN_POS;
		intake.targetIntakeSpeed = 0;
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
