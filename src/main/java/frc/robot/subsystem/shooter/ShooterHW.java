package frc.robot.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterHW {
	public static final double FLYWHEEL_kS = 0.24333;
	public static final double FLYWHEEL_kV = 0.11494;
	public static final double FLYWHEEL_kP = 0.17662 * 2;
	public static final double FLYWHEEL_kD = 0.0;
	public static final double FLYWHEEL_SUPPLY_CURRENT = 40.0;
	public static final double FLYWHEEL_STATOR_CURRENT = 100.0;

	public TalonFX flywheelTalonFX;
	public TalonFX flywheelFollowerTalonFX;

	public VelocityVoltage flywheelRequest = new VelocityVoltage(0).withSlot(0);

	// Cached StatusSignals — flywheel
	public StatusSignal<AngularVelocity> flywheelVelocity;
	public StatusSignal<Voltage> flywheelMotorVoltage;
	public StatusSignal<Current> flywheelStatorCurrent;
	public StatusSignal<Voltage> flywheelSupplyVoltage;
	public StatusSignal<Current> flywheelSupplyCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		// --- Flywheel leader ---
		flywheelTalonFX = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID);
		TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
		flywheelConfig.CurrentLimits.SupplyCurrentLimit = FLYWHEEL_SUPPLY_CURRENT;
		flywheelConfig.CurrentLimits.StatorCurrentLimit = FLYWHEEL_STATOR_CURRENT;
		flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		flywheelConfig.Slot0.kS = FLYWHEEL_kS;
		flywheelConfig.Slot0.kV = FLYWHEEL_kV;
		flywheelConfig.Slot0.kP = FLYWHEEL_kP;
		flywheelConfig.Slot0.kD = FLYWHEEL_kD;
		flywheelTalonFX.getConfigurator().apply(flywheelConfig);
		flywheelTalonFX.clearStickyFaults();
		flywheelTalonFX.setNeutralMode(NeutralModeValue.Coast);

		// --- Flywheel follower ---
		flywheelFollowerTalonFX = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_MOTOR_ID);
		TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
		flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit = FLYWHEEL_SUPPLY_CURRENT;
		flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = FLYWHEEL_STATOR_CURRENT;
		flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		flywheelFollowerTalonFX.getConfigurator().apply(flywheelFollowerConfig);
		flywheelFollowerTalonFX.clearStickyFaults();
		flywheelFollowerTalonFX.setNeutralMode(NeutralModeValue.Coast);
		flywheelFollowerTalonFX.setControl(
				new Follower(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));

		// --- Cache StatusSignals ---
		flywheelVelocity = flywheelTalonFX.getVelocity(false);
		flywheelMotorVoltage = flywheelTalonFX.getMotorVoltage(false);
		flywheelStatorCurrent = flywheelTalonFX.getStatorCurrent(false);
		flywheelSupplyVoltage = flywheelTalonFX.getSupplyVoltage(false);
		flywheelSupplyCurrent = flywheelTalonFX.getSupplyCurrent(false);

		// --- Update frequencies ---
		flywheelVelocity.setUpdateFrequency(50);
		flywheelMotorVoltage.setUpdateFrequency(50);
		flywheelStatorCurrent.setUpdateFrequency(50);
		flywheelSupplyVoltage.setUpdateFrequency(10);
		flywheelSupplyCurrent.setUpdateFrequency(10);

		flywheelTalonFX.optimizeBusUtilization();
		flywheelFollowerTalonFX.optimizeBusUtilization();
	}

	public void sense(ShooterInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(
				flywheelVelocity,
				flywheelMotorVoltage,
				flywheelStatorCurrent,
				flywheelSupplyVoltage,
				flywheelSupplyCurrent);

		inputs.flywheelVelocityRPM = flywheelVelocity.getValueAsDouble() * 60.0;
		inputs.flywheelVoltageVolts = flywheelMotorVoltage.getValueAsDouble();
		inputs.flywheelCurrentAmps = flywheelStatorCurrent.getValueAsDouble();
		inputs.flywheelBusVoltageVolts = flywheelSupplyVoltage.getValueAsDouble();
		inputs.flywheelBusCurrentAmps = flywheelSupplyCurrent.getValueAsDouble();
		inputs.flywheelTsSec = flywheelVelocity.getTimestamp().getTime();
	}

	public void actuate(ShooterInputs inputs, double flywheelRPM) {
		Logger.recordOutput("/Shooter/flywheelErrorRPM", inputs.flywheelVelocityRPM - flywheelRPM);

		if (!Robot.isReal()) return;

		if (flywheelRPM > 0) {
			flywheelTalonFX.setControl(flywheelRequest.withVelocity(flywheelRPM / 60.0));
		} else {
			flywheelTalonFX.setControl(new VoltageOut(0));
		}
	}
}
