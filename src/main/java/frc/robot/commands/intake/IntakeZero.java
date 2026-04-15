package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.IntakeSubsystem;

public class IntakeZero extends Command {
	private static final double TIMEOUT_SECONDS = 3.0;
	private final Timer timer = new Timer();

	public IntakeZero() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.pivotSetpoint = new edu.wpi.first.math.trajectory.TrapezoidProfile.State(
				intake.inputs.pivotPosRadians, 0);
		intake.pivotPID.reset();
		timer.restart();
	}

	@Override
	public void execute() {
		intake.targetPivotPositionRadians = Math.toRadians(IntakeSubsystem.ZERO_GOAL_DEG);
		intake.targetPivotFeedforwardBiasVolts = 0;
		intake.targetIntakeVoltage = 0;
	}

	@Override
	public boolean isFinished() {
		double goal = Math.toRadians(IntakeSubsystem.ZERO_GOAL_DEG);
		boolean atGoal = intake.pivotSetpoint.position == goal && intake.pivotSetpoint.velocity == 0.0;
		return atGoal || timer.hasElapsed(TIMEOUT_SECONDS);
	}

	@Override
	public void end(boolean interrupted) {
		IntakeSubsystem.hasZeroed = true;
	}
}
