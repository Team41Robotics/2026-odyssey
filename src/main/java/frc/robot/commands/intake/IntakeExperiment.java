package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeExperiment extends Command {
	public IntakeExperiment() {
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.targetPivotExperimentRadians = Math.toRadians(intake.experimentSetpointDeg.get());
		intake.targetPivotFeedforwardBiasVolts = 0;
		intake.targetIntakeVoltage = 0;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetPivotExperimentRadians = null;
		intake.targetPivotVoltage = 0;
		intake.targetPivotFeedforwardBiasVolts = 0;
		intake.targetIntakeVoltage = 0;
	}
}
