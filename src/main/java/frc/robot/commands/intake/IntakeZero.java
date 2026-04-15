package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.IntakeSubsystem;

public class IntakeZero extends Command {
	public IntakeZero() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.pivotSetpoint = new edu.wpi.first.math.trajectory.TrapezoidProfile.State(
				intake.inputs.pivotPosRadians, 0);
		intake.pivotPID.reset();
	}

	@Override
	public void execute() {
		intake.targetPivotPositionRadians = Math.toRadians(intake.zeroGoalDeg.get());
		intake.targetPivotFeedforwardBiasVolts = 0;
		intake.targetIntakeVoltage = 0;
	}

	@Override
	public boolean isFinished() {
		double goal = Math.toRadians(intake.zeroGoalDeg.get());
		return intake.pivotSetpoint.position == goal && intake.pivotSetpoint.velocity == 0.0;
	}

	@Override
	public void end(boolean interrupted) {
		IntakeSubsystem.hasZeroed = true;
	}
}
