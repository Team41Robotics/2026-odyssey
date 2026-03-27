package frc.robot.subsystem.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
	public static final double FLYWHEEL_THRES = 200;

	public ShooterHW hw = new ShooterHW();
	public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	public double targetFlywheelRPM = 0;
	public boolean onTarget = false;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Shooter", inputs);

		double flywheelRPMError = inputs.flywheelVelocityRPM - targetFlywheelRPM;
		onTarget = Math.abs(flywheelRPMError) < FLYWHEEL_THRES;

		Logger.recordOutput("/Shooter/targetFlywheelRPM", targetFlywheelRPM);
		Logger.recordOutput("/Shooter/flywheelRPMError", flywheelRPMError);
		Logger.recordOutput("/Shooter/onTarget", onTarget);
		Logger.recordOutput("/Shooter/flywheelSpinningUp", targetFlywheelRPM > 0 && !onTarget);
	}

	public void actuate() {
		hw.actuate(inputs, targetFlywheelRPM);
	}

	public void setShooterSpeed(double rpm) {
		targetFlywheelRPM = rpm;
	}

	public void stopShooter() {
		targetFlywheelRPM = 0;
	}
}
