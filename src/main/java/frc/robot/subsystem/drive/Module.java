package frc.robot.subsystem.drive;

import static edu.wpi.first.math.MathUtil.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
	private static final double WHEEL_CIRCUMFERENCE_METERS = 2.0 * Math.PI * 0.0508;

	private final ModuleIO io;
	public final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private final int index;
	private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

	private final Alert driveDisconnectedAlert;
	private final Alert turnDisconnectedAlert;
	private final Alert turnEncoderDisconnectedAlert;
	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

	public Module(
			ModuleIO io,
			int index,
			SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
		this.io = io;
		this.index = index;
		this.constants = constants;
		driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
		turnDisconnectedAlert = new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
		turnEncoderDisconnectedAlert =
				new Alert("Disconnected turn encoder on module " + index + ".", AlertType.kError);
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Drive/Module" + index, inputs);

		int sampleCount = inputs.odometryTimestamps.length;
		odometryPositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
			Rotation2d angle = inputs.odometryTurnPositions[i];
			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}

		driveDisconnectedAlert.set(!inputs.driveConnected);
		turnDisconnectedAlert.set(!inputs.turnConnected);
		turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
	}

	/** Runs the module with the specified setpoint state (mutates the state to optimize). */
	public void runSetpoint(SwerveModuleState state) {
		state.optimize(getAngle());

		// cos² correction: reduce drive speed proportionally when steer is misaligned, preserving sign
		double targetAng = state.angle.getRadians();
		double cosErr = cos(angleModulus(inputs.turnPosition.getRadians() - targetAng));
		double targetVelRPS = (state.speedMetersPerSecond / WHEEL_CIRCUMFERENCE_METERS) * cosErr * abs(cosErr);

		// setDriveVelocityWithFF takes rad/s — convert wheel_rps → rad/s for the interface
		io.setDriveVelocity(Units.rotationsToRadians(targetVelRPS));
		io.setTurnPosition(state.angle);

		Logger.recordOutput("Drive/Module" + index + "/targetVelRPS", targetVelRPS);
	}

	/** Runs the drive open-loop while holding 0° turn (for SysId characterization). */
	public void runCharacterization(double output) {
		io.setDriveOpenLoop(output);
		io.setTurnPosition(Rotation2d.kZero);
	}

	public void stop() {
		io.setDriveOpenLoop(0.0);
		io.setTurnOpenLoop(0.0);
	}

	public void setBrakeMode(boolean enabled) {
		io.setDriveBrakeMode(enabled);
	}

	public Rotation2d getAngle() {
		return inputs.turnPosition;
	}

	public double getPositionMeters() {
		return inputs.drivePositionRad * constants.WheelRadius;
	}

	public double getVelocityMetersPerSec() {
		return inputs.driveVelocityRadPerSec * constants.WheelRadius;
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	public SwerveModulePosition[] getOdometryPositions() {
		return odometryPositions;
	}

	public double[] getOdometryTimestamps() {
		return inputs.odometryTimestamps;
	}

	/** Returns drive position in radians (for wheel radius characterization). */
	public double getWheelRadiusCharacterizationPosition() {
		return inputs.drivePositionRad;
	}

	/** Returns drive velocity in rotations/sec (Phoenix native units, for FF characterization). */
	public double getFFCharacterizationVelocity() {
		return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
	}
}
