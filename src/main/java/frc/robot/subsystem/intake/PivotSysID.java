package frc.robot.subsystem.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class PivotSysID {
	public SysIdRoutine routine;

	public MutVoltage voltage = Volts.mutable(0);
	public MutAngle position = Radians.mutable(0);
	public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

	public void actuate(Voltage volts) {
		intake.hw.pivotTalonFX.setVoltage(volts.magnitude());
	}

	public void log(SysIdRoutineLog log) {
		log.motor("pivot")
				.voltage(voltage.mut_replace(intake.inputs.pivotVoltageVolts, Volts))
				.angularPosition(position.mut_replace(intake.inputs.pivotPosRadians, Radians))
				.angularVelocity(velocity.mut_replace(intake.inputs.pivotVelRadiansPerSec, RadiansPerSecond));
	}

	public void init() {
		intake.hw.sysIdPivot = true;

		SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(4), Seconds.of(10));
		SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::actuate, this::log, intake);
		routine = new SysIdRoutine(config, mechanism);

		controls.sysidQuasiForward().whileTrue(routine.quasistatic(Direction.kForward));
		controls.sysidQuasiBackward().whileTrue(routine.quasistatic(Direction.kReverse));
		controls.sysidDynaForward().whileTrue(routine.dynamic(Direction.kForward));
		controls.sysidDynaBackward().whileTrue(routine.dynamic(Direction.kReverse));
	}
}
