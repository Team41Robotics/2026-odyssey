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
	public static final double INTAKE_VOLTAGE = 6.0; // FIXME
	public static final double INTAKE_REVERSE_VOLTAGE = -6.0; // FIXME

	public static final double PIVOT_VOLTAGE = 6.0; // FIXME.
	public static final double PIVOT_REVERSE_VOLTAGE = -6.0; // FIXME

	public static final double INTAKE_SUPPLY_CURRENT = 40.0;
	public static final double INTAKE_STATOR_CURRENT = 100.0;

	public static final double PIVOT_SUPPLY_CURRENT = 20.0;
	public static final double PIVOT_STATOR_CURRENT = 40.0;	

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

	public void init() {
		if (!Robot.isReal()) return;

		// --- Pivot ---
		pivotTalonFX = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
		TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
		pivotConfig.CurrentLimits.SupplyCurrentLimit = PIVOT_SUPPLY_CURRENT;
		pivotConfig.CurrentLimits.StatorCurrentLimit = PIVOT_STATOR_CURRENT;
		pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		pivotTalonFX.getConfigurator().apply(pivotConfig);
		pivotTalonFX.clearStickyFaults();
		pivotTalonFX.setNeutralMode(NeutralModeValue.Brake);
		pivotTalonFX.setPosition(0);


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
		pivotPosition = pivotTalonFX.getPosition(false);
		pivotVelocity = pivotTalonFX.getVelocity(false);
		pivotMotorVoltage = pivotTalonFX.getMotorVoltage(false);
		pivotStatorCurrent = pivotTalonFX.getStatorCurrent(false);
		pivotSupplyVoltage = pivotTalonFX.getSupplyVoltage(false);
		pivotSupplyCurrent = pivotTalonFX.getSupplyCurrent(false);

		intakeMotorVoltage = intakeTalonFX.getMotorVoltage(false);
		intakeStatorCurrent = intakeTalonFX.getStatorCurrent(false);

		// --- Update frequencies ---
		pivotPosition.setUpdateFrequency(50);
		pivotVelocity.setUpdateFrequency(50);
		pivotMotorVoltage.setUpdateFrequency(50);
		pivotStatorCurrent.setUpdateFrequency(50);
		pivotSupplyVoltage.setUpdateFrequency(10);
		pivotSupplyCurrent.setUpdateFrequency(10);

		intakeMotorVoltage.setUpdateFrequency(50);
		intakeStatorCurrent.setUpdateFrequency(50);

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
				intakeStatorCurrent);

		inputs.pivotPosRadians = pivotPosition.getValueAsDouble();
		inputs.pivotVelRadiansPerSec = pivotVelocity.getValueAsDouble();
		inputs.pivotVoltageVolts = pivotMotorVoltage.getValueAsDouble();
		inputs.pivotCurrentAmps = pivotStatorCurrent.getValueAsDouble();
		inputs.pivotBusVoltageVolts = pivotSupplyVoltage.getValueAsDouble();
		inputs.pivotBusCurrentAmps = pivotSupplyCurrent.getValueAsDouble();

		inputs.intakeVoltageVolts = intakeMotorVoltage.getValueAsDouble();
		inputs.intakeCurrentAmps = intakeStatorCurrent.getValueAsDouble();
	}

	public void actuate(IntakeInputs inputs, double pivotVoltage, double intakeVoltage) {
		Logger.recordOutput("/Intake/targetPivotVoltage", pivotVoltage);
		Logger.recordOutput("/Intake/targetIntakeVoltage", intakeVoltage);

		if (!Robot.isReal()) return;

		pivotTalonFX.setControl(pivotVoltageRequest.withOutput(pivotVoltage));
		intakeTalonFX.setControl(intakeVoltageRequest.withOutput(intakeVoltage));
	}
}
