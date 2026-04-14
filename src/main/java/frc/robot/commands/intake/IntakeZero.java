package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.IntakeSubsystem;

public class IntakeZero extends Command {
	public static final double ZERO_START_POS_DEG = -90.0 - 11.0 - 20.0; // -121°
	public static final double GOAL_TOLERANCE_DEG = 2.0;

	public IntakeZero() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.hw.seedPivotPosition(Math.toRadians(ZERO_START_POS_DEG));
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
		return intake.inputs.pivotPosRadians >= goal - Math.toRadians(GOAL_TOLERANCE_DEG);
	}

	@Override
	public void end(boolean interrupted) {
		IntakeSubsystem.hasZeroed = true;
	}
}
