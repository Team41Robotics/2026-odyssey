package frc.robot.subsystem.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeHW {
	public static final double EXTEND_OUT_VOLTAGE = 3.0;
	public static final double EXTEND_IN_VOLTAGE = -1.0;

	public static final double INTAKE_VOLTAGE = 5.0; // FIXME.
	public static final double INTAKE_REVERSE_VOLTAGE = -6.0; // FIXME. during deploy

	public static final double EXTENSION_SUPPLY_CURRENT = 60.0;
	public static final double EXTENSION_STATOR_CURRENT = 80.0;
	public static final double INTAKE_SUPPLY_CURRENT = 30.0;
	public static final double INTAKE_STATOR_CURRENT = 60.0;

	public TalonFX intakeTalonFX;
	public TalonFX extensionTalonFX;
	public TalonFX extensionFollowerTalonFX;

	public VoltageOut extendVoltageRequest = new VoltageOut(0);
	public VoltageOut intakeVoltageRequest = new VoltageOut(0);

	// Cached StatusSignals — extension
	public StatusSignal<Angle> extensionPosition;
	public StatusSignal<AngularVelocity> extensionVelocity;
	public StatusSignal<Voltage> extensionMotorVoltage;
	public StatusSignal<Current> extensionStatorCurrent;
	public StatusSignal<Voltage> extensionSupplyVoltage;
	public StatusSignal<Current> extensionSupplyCurrent;

	// Cached StatusSignals — intake (voltage/current only)
	public StatusSignal<Voltage> intakeMotorVoltage;
	public StatusSignal<Current> intakeStatorCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		// --- Extension leader ---
		extensionTalonFX = new TalonFX(IntakeConstants.EXTENSION_MOTOR_ID);
		TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
		extensionConfig.CurrentLimits.SupplyCurrentLimit = EXTENSION_SUPPLY_CURRENT;
		extensionConfig.CurrentLimits.StatorCurrentLimit = EXTENSION_STATOR_CURRENT;
		extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		extensionTalonFX.getConfigurator().apply(extensionConfig);
		extensionTalonFX.clearStickyFaults();
		extensionTalonFX.setNeutralMode(NeutralModeValue.Brake);
		extensionTalonFX.setPosition(0);

		// --- Extension follower ---
		extensionFollowerTalonFX = new TalonFX(IntakeConstants.EXTENSION_FOLLOWER_MOTOR_ID);
		TalonFXConfiguration extensionFollowerConfig = new TalonFXConfiguration();
		extensionFollowerConfig.CurrentLimits.SupplyCurrentLimit = EXTENSION_SUPPLY_CURRENT;
		extensionFollowerConfig.CurrentLimits.StatorCurrentLimit = EXTENSION_STATOR_CURRENT;
		extensionFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		extensionFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		extensionFollowerTalonFX.getConfigurator().apply(extensionFollowerConfig);
		extensionFollowerTalonFX.clearStickyFaults();
		extensionFollowerTalonFX.setNeutralMode(NeutralModeValue.Brake);
		extensionFollowerTalonFX.setControl(
				new Follower(IntakeConstants.EXTENSION_MOTOR_ID, MotorAlignmentValue.Opposed));

		// --- Intake roller ---
		intakeTalonFX = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		intakeConfig.CurrentLimits.SupplyCurrentLimit = INTAKE_SUPPLY_CURRENT;
		intakeConfig.CurrentLimits.StatorCurrentLimit = INTAKE_STATOR_CURRENT;
		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeTalonFX.getConfigurator().apply(intakeConfig);
		intakeTalonFX.clearStickyFaults();
		intakeTalonFX.setNeutralMode(NeutralModeValue.Coast);

		// --- Cache StatusSignals ---
		extensionPosition = extensionTalonFX.getPosition(false);
		extensionVelocity = extensionTalonFX.getVelocity(false);
		extensionMotorVoltage = extensionTalonFX.getMotorVoltage(false);
		extensionStatorCurrent = extensionTalonFX.getStatorCurrent(false);
		extensionSupplyVoltage = extensionTalonFX.getSupplyVoltage(false);
		extensionSupplyCurrent = extensionTalonFX.getSupplyCurrent(false);

		intakeMotorVoltage = intakeTalonFX.getMotorVoltage(false);
		intakeStatorCurrent = intakeTalonFX.getStatorCurrent(false);

		// --- Update frequencies ---
		extensionPosition.setUpdateFrequency(50);
		extensionVelocity.setUpdateFrequency(50);
		extensionMotorVoltage.setUpdateFrequency(50);
		extensionStatorCurrent.setUpdateFrequency(50);
		extensionSupplyVoltage.setUpdateFrequency(10);
		extensionSupplyCurrent.setUpdateFrequency(10);

		intakeMotorVoltage.setUpdateFrequency(50);
		intakeStatorCurrent.setUpdateFrequency(50);

		extensionTalonFX.optimizeBusUtilization();
		extensionFollowerTalonFX.optimizeBusUtilization();
		intakeTalonFX.optimizeBusUtilization();
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
				intakeStatorCurrent);

		inputs.extensionPosRadians = extensionPosition.getValueAsDouble();
		inputs.extensionVelRadiansPerSec = extensionVelocity.getValueAsDouble();
		inputs.extensionVoltageVolts = extensionMotorVoltage.getValueAsDouble();
		inputs.extensionCurrentAmps = extensionStatorCurrent.getValueAsDouble();
		inputs.extensionBusVoltageVolts = extensionSupplyVoltage.getValueAsDouble();
		inputs.extensionBusCurrentAmps = extensionSupplyCurrent.getValueAsDouble();
		inputs.extensionTsSec = extensionPosition.getTimestamp().getTime();

		inputs.intakeVoltageVolts = intakeMotorVoltage.getValueAsDouble();
		inputs.intakeCurrentAmps = intakeStatorCurrent.getValueAsDouble();
	}

	public void actuate(IntakeInputs inputs, double extendVoltage, double intakeVoltage) {
		Logger.recordOutput("/Intake/targetExtendVoltage", extendVoltage);
		Logger.recordOutput("/Intake/targetIntakeVoltage", intakeVoltage);

		if (!Robot.isReal()) return;

		extensionTalonFX.setControl(extendVoltageRequest.withOutput(extendVoltage));
		intakeTalonFX.setControl(intakeVoltageRequest.withOutput(intakeVoltage));
	}
}
