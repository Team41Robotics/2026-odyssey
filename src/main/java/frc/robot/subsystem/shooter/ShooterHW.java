package frc.robot.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterHW {
	public static final double FLYWHEEL_kS = 0.24333;
	public static final double FLYWHEEL_kV = 0.11494;
	public static final double FLYWHEEL_kP = 0.17662;
	public static final double FLYWHEEL_kD = 0.0;
	public static final double FLYWHEEL_SUPPLY_CURRENT = 30.0; // FIXME. (A)
	public static final double FLYWHEEL_STATOR_CURRENT = 60.0; // FIXME. (A)

	public static final double ELEVATOR_kS = 0.1; // FIXME.
	public static final double ELEVATOR_kP = 1.0; // FIXME.

	public TalonFX flywheelTalonFX;
	public TalonFX flywheelFollowerTalonFX;
	public TalonFX elevatorTalonFX;

	public VelocityVoltage flywheelRequest = new VelocityVoltage(0).withSlot(0);
	public PositionVoltage elevatorRequest = new PositionVoltage(0).withSlot(0);

	// Cached StatusSignals — flywheel
	public StatusSignal<AngularVelocity> flywheelVelocity;
	public StatusSignal<Voltage> flywheelMotorVoltage;
	public StatusSignal<Current> flywheelStatorCurrent;
	public StatusSignal<Voltage> flywheelSupplyVoltage;
	public StatusSignal<Current> flywheelSupplyCurrent;

	// Cached StatusSignals — elevator
	public StatusSignal<Angle> elevatorPosition;
	public StatusSignal<AngularVelocity> elevatorVelocity;
	public StatusSignal<Voltage> elevatorMotorVoltage;
	public StatusSignal<Current> elevatorStatorCurrent;
	public StatusSignal<Voltage> elevatorSupplyVoltage;
	public StatusSignal<Current> elevatorSupplyCurrent;

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
		flywheelFollowerTalonFX.getConfigurator().apply(new TalonFXConfiguration());
		flywheelFollowerTalonFX.clearStickyFaults();
		flywheelFollowerTalonFX.setNeutralMode(NeutralModeValue.Coast);
		flywheelFollowerTalonFX.setControl(
				new Follower(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));

		// --- Elevator ---
		elevatorTalonFX = new TalonFX(ShooterConstants.ELEVATOR_MOTOR_ID);
		TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
		elevatorConfig.Slot0.kS = ELEVATOR_kS;
		elevatorConfig.Slot0.kP = ELEVATOR_kP;
		elevatorTalonFX.getConfigurator().apply(elevatorConfig);
		elevatorTalonFX.clearStickyFaults();
		elevatorTalonFX.setNeutralMode(NeutralModeValue.Brake);
		elevatorTalonFX.setPosition(0); // FIXME. seed to absolute position if encoder available

		// --- Cache StatusSignals ---
		flywheelVelocity = flywheelTalonFX.getVelocity(false);
		flywheelMotorVoltage = flywheelTalonFX.getMotorVoltage(false);
		flywheelStatorCurrent = flywheelTalonFX.getStatorCurrent(false);
		flywheelSupplyVoltage = flywheelTalonFX.getSupplyVoltage(false);
		flywheelSupplyCurrent = flywheelTalonFX.getSupplyCurrent(false);

		elevatorPosition = elevatorTalonFX.getPosition(false);
		elevatorVelocity = elevatorTalonFX.getVelocity(false);
		elevatorMotorVoltage = elevatorTalonFX.getMotorVoltage(false);
		elevatorStatorCurrent = elevatorTalonFX.getStatorCurrent(false);
		elevatorSupplyVoltage = elevatorTalonFX.getSupplyVoltage(false);
		elevatorSupplyCurrent = elevatorTalonFX.getSupplyCurrent(false);

		// --- Update frequencies ---
		flywheelVelocity.setUpdateFrequency(50);
		flywheelMotorVoltage.setUpdateFrequency(50);
		flywheelStatorCurrent.setUpdateFrequency(50);
		flywheelSupplyVoltage.setUpdateFrequency(10);
		flywheelSupplyCurrent.setUpdateFrequency(10);

		elevatorPosition.setUpdateFrequency(50);
		elevatorVelocity.setUpdateFrequency(50);
		elevatorMotorVoltage.setUpdateFrequency(50);
		elevatorStatorCurrent.setUpdateFrequency(50);
		elevatorSupplyVoltage.setUpdateFrequency(10);
		elevatorSupplyCurrent.setUpdateFrequency(10);

		flywheelTalonFX.optimizeBusUtilization();
		flywheelFollowerTalonFX.optimizeBusUtilization();
		elevatorTalonFX.optimizeBusUtilization();
	}

	public void sense(ShooterInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(
				flywheelVelocity,
				flywheelMotorVoltage,
				flywheelStatorCurrent,
				flywheelSupplyVoltage,
				flywheelSupplyCurrent,
				elevatorPosition,
				elevatorVelocity,
				elevatorMotorVoltage,
				elevatorStatorCurrent,
				elevatorSupplyVoltage,
				elevatorSupplyCurrent);

		inputs.flywheelVelocityRPM = flywheelVelocity.getValueAsDouble() * 60.0;
		inputs.flywheelVoltageVolts = flywheelMotorVoltage.getValueAsDouble();
		inputs.flywheelCurrentAmps = flywheelStatorCurrent.getValueAsDouble();
		inputs.flywheelBusVoltageVolts = flywheelSupplyVoltage.getValueAsDouble();
		inputs.flywheelBusCurrentAmps = flywheelSupplyCurrent.getValueAsDouble();
		inputs.flywheelTsSec = flywheelVelocity.getTimestamp().getTime();

		inputs.elevatorPosRotations = elevatorPosition.getValueAsDouble();
		inputs.elevatorVelRotationsPerSec = elevatorVelocity.getValueAsDouble();
		inputs.elevatorVoltageVolts = elevatorMotorVoltage.getValueAsDouble();
		inputs.elevatorCurrentAmps = elevatorStatorCurrent.getValueAsDouble();
		inputs.elevatorBusVoltageVolts = elevatorSupplyVoltage.getValueAsDouble();
		inputs.elevatorBusCurrentAmps = elevatorSupplyCurrent.getValueAsDouble();
		inputs.elevatorTsSec = elevatorPosition.getTimestamp().getTime();
	}

	public void actuate(ShooterInputs inputs, double flywheelRPM, double elevatorPos) {
		Logger.recordOutput("/Shooter/flywheelErrorRPM", inputs.flywheelVelocityRPM - flywheelRPM);
		Logger.recordOutput("/Shooter/elevatorErrorRotations", inputs.elevatorPosRotations - elevatorPos);

		if (!Robot.isReal()) return;

		if (flywheelRPM > 0) {
			flywheelTalonFX.setControl(flywheelRequest.withVelocity(flywheelRPM / 60.0));
		} else {
			flywheelTalonFX.setControl(new DutyCycleOut(0));
		}
		elevatorTalonFX.setControl(elevatorRequest.withPosition(elevatorPos));
	}

	/** Manual elevator override (duty cycle). Bypasses position control. */
	public void setElevatorSpeed(double speed) {
		if (!Robot.isReal()) return;
		elevatorTalonFX.setControl(new DutyCycleOut(speed));
	}
}
