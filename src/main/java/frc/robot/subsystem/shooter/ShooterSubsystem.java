package frc.robot.subsystem.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
	public static final double FLYWHEEL_THRES = 200; // RPM // FIXME.
	public static final double ELEVATOR_POS_THRES = 0.5; // rotations // FIXME.
	public static final double IDLE_RPM = 2000; // FIXME.

	public ShooterHW hw = new ShooterHW();
	public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	public double targetFlywheelRPM = 0;
	public double targetElevatorPos = 0; // motor rotations
	public boolean onTarget = false;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Shooter", inputs);

		onTarget = Math.abs(inputs.flywheelVelocityRPM - targetFlywheelRPM) < FLYWHEEL_THRES
				&& Math.abs(inputs.elevatorPosRotations - targetElevatorPos) < ELEVATOR_POS_THRES;

		Logger.recordOutput("/Shooter/targetFlywheelRPM", targetFlywheelRPM);
		Logger.recordOutput("/Shooter/targetElevatorPos", targetElevatorPos);
		Logger.recordOutput("/Shooter/onTarget", onTarget);
	}

	public void actuate() {
		hw.actuate(inputs, targetFlywheelRPM, targetElevatorPos);
	}

	public void setShooterSpeed(double rpm) {
		targetFlywheelRPM = rpm;
	}

	public void setElevatorSpeed(double speed) {
		hw.setElevatorSpeed(speed);
	}

	public void stopShooter() {
		targetFlywheelRPM = 0;
	}
}
