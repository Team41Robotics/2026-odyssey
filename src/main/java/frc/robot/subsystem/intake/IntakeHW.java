package frc.robot.subsystem.intake;

import static frc.robot.util.PhoenixUtil.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;

public class IntakeHW {
	public static final double PIVOT_GEAR_RATIO = 14.0;

	public TalonFX intakeTalonFX;
	public TalonFX pivotTalonFX;

	public VoltageOut pivotVoltageRequest = new VoltageOut(0);
	public VoltageOut intakeVoltageRequest = new VoltageOut(0);

	// Cached StatusSignals — pivot
	public StatusSignal<Angle> pivotPosition;
	public StatusSignal<AngularVelocity> pivotVelocity;
	public StatusSignal<Voltage> pivotMotorVoltage;
	public StatusSignal<Current> pivotStatorCurrent;
	public StatusSignal<Voltage> pivotSupplyVoltage;
	public StatusSignal<Current> pivotSupplyCurrent;

	// Cached StatusSignals — intake
	public StatusSignal<Voltage> intakeMotorVoltage;
	public StatusSignal<Current> intakeStatorCurrent;
	public StatusSignal<AngularVelocity> intakeVelocity;
	public StatusSignal<Voltage> intakeSupplyVoltage;
	public StatusSignal<Current> intakeSupplyCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		// --- Pivot ---
		pivotTalonFX = new TalonFX(21);
		TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
		pivotConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		pivotConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		pivotConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
		tryUntilOk(5, () -> pivotTalonFX.getConfigurator().apply(pivotConfig, 0.25));
		tryUntilOk(5, () -> pivotTalonFX.clearStickyFaults(0.25));
		tryUntilOk(5, () -> pivotTalonFX.setNeutralMode(NeutralModeValue.Brake));
		tryUntilOk(5, () -> pivotTalonFX.setPosition(0, 0.25));

		// --- Intake roller ---
		intakeTalonFX = new TalonFX(31);
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		intakeConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
		intakeConfig.CurrentLimits.StatorCurrentLimit = 120.0;
		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		tryUntilOk(5, () -> intakeTalonFX.getConfigurator().apply(intakeConfig, 0.25));
		tryUntilOk(5, () -> intakeTalonFX.clearStickyFaults(0.25));
		tryUntilOk(5, () -> intakeTalonFX.setNeutralMode(NeutralModeValue.Coast));

		// --- Cache StatusSignals ---
		pivotPosition = pivotTalonFX.getPosition(false);
		pivotVelocity = pivotTalonFX.getVelocity(false);
		pivotMotorVoltage = pivotTalonFX.getMotorVoltage(false);
		pivotStatorCurrent = pivotTalonFX.getStatorCurrent(false);
		pivotSupplyVoltage = pivotTalonFX.getSupplyVoltage(false);
		pivotSupplyCurrent = pivotTalonFX.getSupplyCurrent(false);

		intakeMotorVoltage = intakeTalonFX.getMotorVoltage(false);
		intakeStatorCurrent = intakeTalonFX.getStatorCurrent(false);
		intakeVelocity = intakeTalonFX.getVelocity(false);
		intakeSupplyVoltage = intakeTalonFX.getSupplyVoltage(false);
		intakeSupplyCurrent = intakeTalonFX.getSupplyCurrent(false);

		// --- Update frequencies ---
		pivotPosition.setUpdateFrequency(50);
		pivotVelocity.setUpdateFrequency(50);
		pivotMotorVoltage.setUpdateFrequency(50);
		pivotStatorCurrent.setUpdateFrequency(50);
		pivotSupplyVoltage.setUpdateFrequency(50);
		pivotSupplyCurrent.setUpdateFrequency(50);

		intakeMotorVoltage.setUpdateFrequency(50);
		intakeStatorCurrent.setUpdateFrequency(50);
		intakeVelocity.setUpdateFrequency(50);
		intakeSupplyVoltage.setUpdateFrequency(50);
		intakeSupplyCurrent.setUpdateFrequency(50);

		pivotTalonFX.optimizeBusUtilization();
		intakeTalonFX.optimizeBusUtilization();
	}

	public void sense(IntakeInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(
				pivotPosition,
				pivotVelocity,
				pivotMotorVoltage,
				pivotStatorCurrent,
				pivotSupplyVoltage,
				pivotSupplyCurrent,
				intakeMotorVoltage,
				intakeStatorCurrent,
				intakeVelocity,
				intakeSupplyVoltage,
				intakeSupplyCurrent);

		inputs.pivotPosRadians = pivotPosition.getValueAsDouble() * 2.0 * PI;
		inputs.pivotVelRadiansPerSec = pivotVelocity.getValueAsDouble() * 2.0 * PI;
		inputs.pivotVoltageVolts = pivotMotorVoltage.getValueAsDouble();
		inputs.pivotCurrentAmps = pivotStatorCurrent.getValueAsDouble();
		inputs.pivotBusVoltageVolts = pivotSupplyVoltage.getValueAsDouble();
		inputs.pivotBusCurrentAmps = pivotSupplyCurrent.getValueAsDouble();

		inputs.intakeVoltageVolts = intakeMotorVoltage.getValueAsDouble();
		inputs.intakeCurrentAmps = intakeStatorCurrent.getValueAsDouble();
		inputs.rollerVelocityRPS = intakeVelocity.getValueAsDouble();
		inputs.rollerBusVoltageVolts = intakeSupplyVoltage.getValueAsDouble();
		inputs.rollerBusCurrentAmps = intakeSupplyCurrent.getValueAsDouble();

		inputs.pivotTsSec = pivotSupplyVoltage.getTimestamp().getTime();
		inputs.rollerTsSec = intakeSupplyVoltage.getTimestamp().getTime();
	}

	public void actuate(IntakeInputs inputs, double pivotVoltage, double intakeVoltage) {
		if (!Robot.isReal()) return;

		pivotTalonFX.setControl(pivotVoltageRequest.withOutput(pivotVoltage));
		intakeTalonFX.setControl(intakeVoltageRequest.withOutput(intakeVoltage));
	}

	public void zeroPivotForced() {
		if (!Robot.isReal()) return;
		tryUntilOk(5, () -> pivotTalonFX.setPosition(0, 0.25));
	}
}
