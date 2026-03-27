package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.IntakeHW;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("static-access")
public class ExtendOut extends Command {
	public double deployReverseEndTime = 0;

	public ExtendOut() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		deployReverseEndTime = Timer.getFPGATimestamp() + IntakeHW.INTAKE_REVERSE_DURATION;
		intake.extendTarget = IntakeHW.EXTEND_OUT_POS;
	}

	@Override
	public void execute() {
		double now = Timer.getFPGATimestamp();
		boolean reversing = now < deployReverseEndTime;
		intake.targetIntakeSpeed = reversing ? IntakeHW.INTAKE_REVERSE_SPEED : IntakeHW.INTAKE_SPEED;
		Logger.recordOutput("/Intake/reversing", reversing);
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetIntakeSpeed = 0;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
