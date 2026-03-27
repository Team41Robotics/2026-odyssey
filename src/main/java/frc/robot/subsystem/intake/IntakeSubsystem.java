package frc.robot.subsystem.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
	public static final TrapezoidProfile EXTEND_PROFILE = new TrapezoidProfile(IntakeHW.EXTEND_CONSTRAINTS);

	public IntakeHW hw = new IntakeHW();
	public IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	// Controller state (belongs here: needed across loops to run the profile)
	public TrapezoidProfile.State extendSetpoint = new State();

	// Per-loop actuator targets written by commands before actuate()
	public double extendTarget = IntakeHW.EXTEND_IN_POS;
	public double targetIntakeSpeed = 0;
	public double targetFeederSpeed = 0;

	public void init() {
		hw.init();
		sense();
		extendSetpoint = new State(inputs.extensionPosRadians, inputs.extensionVelRadiansPerSec);
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Intake", inputs);

		Logger.recordOutput("/Intake/extendTarget", extendTarget);
		Logger.recordOutput("/Intake/extendSetpoint", extendSetpoint.position);
		Logger.recordOutput("/Intake/targetIntakeSpeed", targetIntakeSpeed);
		Logger.recordOutput("/Intake/targetFeederSpeed", targetFeederSpeed);
	}

	public void actuate() {
		State newSetpoint = EXTEND_PROFILE.calculate(LOOP_PERIOD, extendSetpoint, new State(extendTarget, 0));
		double ff = IntakeHW.EXTEND_FF.calculateWithVelocities(extendSetpoint.velocity, newSetpoint.velocity);
		extendSetpoint = newSetpoint;

		hw.actuate(inputs, extendSetpoint.position, ff, targetIntakeSpeed, targetFeederSpeed);
	}

	public void setFeederSpeed(double speed) {
		targetFeederSpeed = speed;
	}
}
