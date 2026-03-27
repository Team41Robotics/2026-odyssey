package frc.robot.subsystem.intake;

import static java.lang.Math.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeHW {
	public static final double EXTEND_kS = 0.1; // FIXME.
	public static final double EXTEND_kV = 0.5; // FIXME.
	public static final double EXTEND_kP = 15.0; // FIXME.
	public static final double EXTEND_RATIO = 1.0 / 12.0; // FIXME. motor rotations → mechanism rotations
	public static final double EXTEND_OUT_POS = 10.0; // FIXME. mechanism rotations extended
	public static final double EXTEND_IN_POS = 0.0; // FIXME. mechanism rotations retracted
	public static final TrapezoidProfile.Constraints EXTEND_CONSTRAINTS =
			new TrapezoidProfile.Constraints(12, 30); // FIXME. (rot/s, rot/s²)
	public static final SimpleMotorFeedforward EXTEND_FF = new SimpleMotorFeedforward(EXTEND_kS, EXTEND_kV);

	public static final double INTAKE_SPEED = 1.0; // FIXME. duty cycle when deployed
	public static final double INTAKE_REVERSE_SPEED = -0.5; // FIXME. duty cycle during deploy reverse
	public static final double INTAKE_REVERSE_DURATION = 0.5; // seconds

	public TalonFX intakeTalonFX;
	public TalonFX extensionTalonFX;
	public TalonFX extensionFollowerTalonFX;
	public TalonFX feederTalonFX;

	public PositionVoltage extendRequest = new PositionVoltage(0).withSlot(0);
	public DutyCycleOut intakeDuty = new DutyCycleOut(0);
	public DutyCycleOut feederDuty = new DutyCycleOut(0);

	// Cached StatusSignals — extension
	public StatusSignal<Angle> extensionPosition;
	public StatusSignal<AngularVelocity> extensionVelocity;
	public StatusSignal<Voltage> extensionMotorVoltage;
	public StatusSignal<Current> extensionStatorCurrent;
	public StatusSignal<Voltage> extensionSupplyVoltage;
	public StatusSignal<Current> extensionSupplyCurrent;

	// Cached StatusSignals — intake + feeder (voltage/current only)
	public StatusSignal<Voltage> intakeMotorVoltage;
	public StatusSignal<Current> intakeStatorCurrent;
	public StatusSignal<Voltage> feederMotorVoltage;
	public StatusSignal<Current> feederStatorCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		// --- Extension leader ---
		extensionTalonFX = new TalonFX(IntakeConstants.EXTENSION_MOTOR_ID);
		TalonFXConfiguration extendConfig = new TalonFXConfiguration();
		extendConfig.Slot0.kP = EXTEND_kP * EXTEND_RATIO * 2 * PI;
		extensionTalonFX.getConfigurator().apply(extendConfig);
		extensionTalonFX.clearStickyFaults();
		extensionTalonFX.setNeutralMode(NeutralModeValue.Brake);
		extensionTalonFX.setPosition(0); // FIXME. seed to absolute position if encoder available

		// --- Extension follower ---
		extensionFollowerTalonFX = new TalonFX(IntakeConstants.EXTENSION_FOLLOWER_MOTOR_ID);
		extensionFollowerTalonFX.getConfigurator().apply(new TalonFXConfiguration());
		extensionFollowerTalonFX.clearStickyFaults();
		extensionFollowerTalonFX.setControl(
				new Follower(IntakeConstants.EXTENSION_MOTOR_ID, MotorAlignmentValue.Opposed));

		// --- Intake roller ---
		intakeTalonFX = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
		intakeTalonFX.getConfigurator().apply(new TalonFXConfiguration());
		intakeTalonFX.clearStickyFaults();
		intakeTalonFX.setNeutralMode(NeutralModeValue.Coast);

		// --- Feeder ---
		feederTalonFX = new TalonFX(IntakeConstants.FEEDER_MOTOR_ID);
		feederTalonFX.getConfigurator().apply(new TalonFXConfiguration());
		feederTalonFX.clearStickyFaults();
		feederTalonFX.setNeutralMode(NeutralModeValue.Coast);

		// --- Cache StatusSignals ---
		extensionPosition = extensionTalonFX.getPosition(false);
		extensionVelocity = extensionTalonFX.getVelocity(false);
		extensionMotorVoltage = extensionTalonFX.getMotorVoltage(false);
		extensionStatorCurrent = extensionTalonFX.getStatorCurrent(false);
		extensionSupplyVoltage = extensionTalonFX.getSupplyVoltage(false);
		extensionSupplyCurrent = extensionTalonFX.getSupplyCurrent(false);

		intakeMotorVoltage = intakeTalonFX.getMotorVoltage(false);
		intakeStatorCurrent = intakeTalonFX.getStatorCurrent(false);
		feederMotorVoltage = feederTalonFX.getMotorVoltage(false);
		feederStatorCurrent = feederTalonFX.getStatorCurrent(false);

		// --- Update frequencies ---
		extensionPosition.setUpdateFrequency(50);
		extensionVelocity.setUpdateFrequency(50);
		extensionMotorVoltage.setUpdateFrequency(50);
		extensionStatorCurrent.setUpdateFrequency(50);
		extensionSupplyVoltage.setUpdateFrequency(10);
		extensionSupplyCurrent.setUpdateFrequency(10);

		intakeMotorVoltage.setUpdateFrequency(50);
		intakeStatorCurrent.setUpdateFrequency(50);
		feederMotorVoltage.setUpdateFrequency(50);
		feederStatorCurrent.setUpdateFrequency(50);

		extensionTalonFX.optimizeBusUtilization();
		extensionFollowerTalonFX.optimizeBusUtilization();
		intakeTalonFX.optimizeBusUtilization();
		feederTalonFX.optimizeBusUtilization();
	}

	public void sense(IntakeInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(
				extensionPosition,
				extensionVelocity,
				extensionMotorVoltage,
				extensionStatorCurrent,
				extensionSupplyVoltage,
				extensionSupplyCurrent,
				intakeMotorVoltage,
				intakeStatorCurrent,
				feederMotorVoltage,
				feederStatorCurrent);

		inputs.extensionPosRadians = extensionPosition.getValueAsDouble() * EXTEND_RATIO * 2 * PI;
		inputs.extensionVelRadiansPerSec = extensionVelocity.getValueAsDouble() * EXTEND_RATIO * 2 * PI;
		inputs.extensionVoltageVolts = extensionMotorVoltage.getValueAsDouble();
		inputs.extensionCurrentAmps = extensionStatorCurrent.getValueAsDouble();
		inputs.extensionBusVoltageVolts = extensionSupplyVoltage.getValueAsDouble();
		inputs.extensionBusCurrentAmps = extensionSupplyCurrent.getValueAsDouble();
		inputs.extensionTsSec = extensionPosition.getTimestamp().getTime();

		inputs.intakeVoltageVolts = intakeMotorVoltage.getValueAsDouble();
		inputs.intakeCurrentAmps = intakeStatorCurrent.getValueAsDouble();
		inputs.feederVoltageVolts = feederMotorVoltage.getValueAsDouble();
		inputs.feederCurrentAmps = feederStatorCurrent.getValueAsDouble();
	}

	/** Drive all intake motors. extensionSetpointRad is in mechanism radians. */
	public void actuate(
			IntakeInputs inputs,
			double extensionSetpointRad,
			double extensionFF,
			double intakeSpeed,
			double feederSpeed) {
		Logger.recordOutput("/Intake/extensionErrorRad", inputs.extensionPosRadians - extensionSetpointRad);

		if (!Robot.isReal()) return;

		extensionTalonFX.setControl(extendRequest
				.withPosition(extensionSetpointRad / (EXTEND_RATIO * 2 * PI))
				.withFeedForward(extensionFF));
		intakeTalonFX.setControl(intakeDuty.withOutput(intakeSpeed));
		feederTalonFX.setControl(feederDuty.withOutput(feederSpeed));
	}
}
