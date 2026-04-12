package frc.robot.commands.intake;

import static frc.robot.RobotContainer.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class ZeroPivot extends Command {
	public ZeroPivot() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.hw.zeroPivot();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
