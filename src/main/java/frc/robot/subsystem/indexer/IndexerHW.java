package frc.robot.subsystem.indexer;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;

public class IndexerHW {
	public TalonFX rollerTalonFX;
	public TalonFX indexerTalonFX;

	public VoltageOut rollerVoltageRequest = new VoltageOut(0);
	public VoltageOut indexerVoltageRequest = new VoltageOut(0);

	// Cached StatusSignals — roller
	public StatusSignal<Voltage> rollerMotorVoltage;
	public StatusSignal<Current> rollerStatorCurrent;
	public StatusSignal<AngularVelocity> rollerVelocity;
	public StatusSignal<Voltage> rollerSupplyVoltage;
	public StatusSignal<Current> rollerSupplyCurrent;

	// Cached StatusSignals — indexer follower
	public StatusSignal<Voltage> indexerMotorVoltage;
	public StatusSignal<Current> indexerStatorCurrent;
	public StatusSignal<AngularVelocity> indexerVelocity;
	public StatusSignal<Voltage> indexerSupplyVoltage;
	public StatusSignal<Current> indexerSupplyCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		// --- Roller ---
		rollerTalonFX = new TalonFX(60);
		TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
		rollerConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
		rollerConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		tryUntilOk(5, () -> rollerTalonFX.getConfigurator().apply(rollerConfig, 0.25));
		tryUntilOk(5, () -> rollerTalonFX.clearStickyFaults(0.25));
		tryUntilOk(5, () -> rollerTalonFX.setNeutralMode(NeutralModeValue.Coast));

		// --- Indexer follows roller ---
		indexerTalonFX = new TalonFX(32);
		TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
		indexerConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
		indexerConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		tryUntilOk(5, () -> indexerTalonFX.getConfigurator().apply(indexerConfig, 0.25));
		tryUntilOk(5, () -> indexerTalonFX.clearStickyFaults(0.25));
		tryUntilOk(5, () -> indexerTalonFX.setNeutralMode(NeutralModeValue.Coast));

		// --- Cache StatusSignals ---
		rollerMotorVoltage = rollerTalonFX.getMotorVoltage(false);
		rollerStatorCurrent = rollerTalonFX.getStatorCurrent(false);
		rollerVelocity = rollerTalonFX.getVelocity(false);
		rollerSupplyVoltage = rollerTalonFX.getSupplyVoltage(false);
		rollerSupplyCurrent = rollerTalonFX.getSupplyCurrent(false);
		indexerMotorVoltage = indexerTalonFX.getMotorVoltage(false);
		indexerStatorCurrent = indexerTalonFX.getStatorCurrent(false);
		indexerVelocity = indexerTalonFX.getVelocity(false);
		indexerSupplyVoltage = indexerTalonFX.getSupplyVoltage(false);
		indexerSupplyCurrent = indexerTalonFX.getSupplyCurrent(false);

		// --- Update frequencies ---
		rollerMotorVoltage.setUpdateFrequency(50);
		rollerStatorCurrent.setUpdateFrequency(50);
		rollerVelocity.setUpdateFrequency(50);
		rollerSupplyVoltage.setUpdateFrequency(50);
		rollerSupplyCurrent.setUpdateFrequency(50);
		indexerMotorVoltage.setUpdateFrequency(50);
		indexerStatorCurrent.setUpdateFrequency(50);
		indexerVelocity.setUpdateFrequency(50);
		indexerSupplyVoltage.setUpdateFrequency(50);
		indexerSupplyCurrent.setUpdateFrequency(50);

		rollerTalonFX.optimizeBusUtilization();
		indexerTalonFX.optimizeBusUtilization();
	}

	public void sense(IndexerInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(
				rollerMotorVoltage,
				rollerStatorCurrent,
				rollerVelocity,
				rollerSupplyVoltage,
				rollerSupplyCurrent,
				indexerMotorVoltage,
				indexerStatorCurrent,
				indexerVelocity,
				indexerSupplyVoltage,
				indexerSupplyCurrent);

		inputs.rollerVoltageVolts = rollerMotorVoltage.getValueAsDouble();
		inputs.rollerCurrentAmps = rollerStatorCurrent.getValueAsDouble();
		inputs.rollerVelocityRPM = rollerVelocity.getValueAsDouble() * 60.0;
		inputs.rollerBusVoltageVolts = rollerSupplyVoltage.getValueAsDouble();
		inputs.rollerBusCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
		inputs.indexerVoltageVolts = indexerMotorVoltage.getValueAsDouble();
		inputs.indexerCurrentAmps = indexerStatorCurrent.getValueAsDouble();
		inputs.indexerVelocityRPM = indexerVelocity.getValueAsDouble() * 60.0;
		inputs.indexerBusVoltageVolts = indexerSupplyVoltage.getValueAsDouble();
		inputs.indexerBusCurrentAmps = indexerSupplyCurrent.getValueAsDouble();
	}

	public void actuate(IndexerInputs inputs, double rollerVoltage, double indexerVoltage) {
		if (!Robot.isReal()) return;

		rollerTalonFX.setControl(rollerVoltageRequest.withOutput(rollerVoltage));
		indexerTalonFX.setControl(indexerVoltageRequest.withOutput(indexerVoltage));
	}
}
