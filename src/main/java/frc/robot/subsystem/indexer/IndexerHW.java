package frc.robot.subsystem.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IndexerHW {
	public static final double ROLLER_SUPPLY_CURRENT = 40.0;
	public static final double ROLLER_STATOR_CURRENT = 80.0;


	public TalonFX rollerTalonFX;
	public TalonFX indexerTalonFX;

	public VoltageOut rollerVoltageRequest = new VoltageOut(0);
	public VoltageOut indexerVoltageRequest = new VoltageOut(0);

	// Cached StatusSignals — roller
	public StatusSignal<Voltage> rollerMotorVoltage;
	public StatusSignal<Current> rollerStatorCurrent;

	// Cached StatusSignals — indexer follower
	public StatusSignal<Voltage> indexerMotorVoltage;
	public StatusSignal<Current> indexerStatorCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		// --- Roller ---
		rollerTalonFX = new TalonFX(IntakeConstants.FEEDER_MOTOR_ID);
		TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
		rollerConfig.CurrentLimits.SupplyCurrentLimit = ROLLER_SUPPLY_CURRENT;
		rollerConfig.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_CURRENT;
		rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		rollerTalonFX.getConfigurator().apply(rollerConfig);
		rollerTalonFX.clearStickyFaults();
		rollerTalonFX.setNeutralMode(NeutralModeValue.Coast);

		// --- Indexer follows roller ---
		indexerTalonFX = new TalonFX(ShooterConstants.ELEVATOR_MOTOR_ID);
		TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
		indexerConfig.CurrentLimits.SupplyCurrentLimit = ROLLER_SUPPLY_CURRENT;
		indexerConfig.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_CURRENT;
		indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		indexerTalonFX.getConfigurator().apply(indexerConfig);
		indexerTalonFX.clearStickyFaults();
		indexerTalonFX.setNeutralMode(NeutralModeValue.Coast);

		// --- Cache StatusSignals ---
		rollerMotorVoltage = rollerTalonFX.getMotorVoltage(false);
		rollerStatorCurrent = rollerTalonFX.getStatorCurrent(false);
		indexerMotorVoltage = indexerTalonFX.getMotorVoltage(false);
		indexerStatorCurrent = indexerTalonFX.getStatorCurrent(false);

		// --- Update frequencies ---
		rollerMotorVoltage.setUpdateFrequency(50);
		rollerStatorCurrent.setUpdateFrequency(50);
		indexerMotorVoltage.setUpdateFrequency(50);
		indexerStatorCurrent.setUpdateFrequency(50);

		rollerTalonFX.optimizeBusUtilization();
		indexerTalonFX.optimizeBusUtilization();
	}

	public void sense(IndexerInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(rollerMotorVoltage, rollerStatorCurrent, indexerMotorVoltage, indexerStatorCurrent);

		inputs.rollerVoltageVolts = rollerMotorVoltage.getValueAsDouble();
		inputs.rollerCurrentAmps = rollerStatorCurrent.getValueAsDouble();
		inputs.indexerVoltageVolts = indexerMotorVoltage.getValueAsDouble();
		inputs.indexerCurrentAmps = indexerStatorCurrent.getValueAsDouble();
	}

	public void actuate(IndexerInputs inputs, double rollerVoltage, double indexerVoltage) {
		Logger.recordOutput("/Indexer/commandedRollerVoltage", rollerVoltage);
		Logger.recordOutput("/Indexer/commandedIndexerVoltage", indexerVoltage);

		if (!Robot.isReal()) return;

		rollerTalonFX.setControl(rollerVoltageRequest.withOutput(rollerVoltage));
		indexerTalonFX.setControl(indexerVoltageRequest.withOutput(indexerVoltage));
	}
}
