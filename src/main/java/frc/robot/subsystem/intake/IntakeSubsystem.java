package frc.robot.subsystem.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeSubsystem extends SubsystemBase {
	public static final double DEFAULT_kP = 15.0;
	public static final double DEFAULT_kD = 0.04;
	public static final double DEFAULT_kG = 1.5;
	public static final double DEFAULT_kV = 1.5;
	public static final double DEFAULT_G_OFFSET_DEG = 15.0;
	public static final double DEFAULT_MAX_VEL = 3.0;
	public static final double DEFAULT_MAX_ACCEL = 6.0;
	public static final double DEFAULT_INTAKE_UP_DEG = 65.0;
	public static final double DEFAULT_ZERO_GOAL_DEG = 20.0;
	public static final double DEFAULT_INTAKE_DOWN_BIAS_VOLTS = 1.0;
	public static final double DEFAULT_FB_DISABLE_LO_DEG = 10.0;
	public static final double DEFAULT_FB_DISABLE_HI_DEG = 45.0;
	public static final double DEFAULT_FB_DISABLE_ABOVE_DEG = 900.0;

	public LoggedNetworkNumber kP = new LoggedNetworkNumber("/Intake/tuning/kP", DEFAULT_kP);
	public LoggedNetworkNumber kD = new LoggedNetworkNumber("/Intake/tuning/kD", DEFAULT_kD);
	public LoggedNetworkNumber kG = new LoggedNetworkNumber("/Intake/tuning/kG", DEFAULT_kG);
	public LoggedNetworkNumber kV = new LoggedNetworkNumber("/Intake/tuning/kV", DEFAULT_kV);
	public LoggedNetworkNumber gOffsetDeg =
			new LoggedNetworkNumber("/Intake/tuning/gOffsetDeg", DEFAULT_G_OFFSET_DEG);
	public LoggedNetworkNumber maxVel = new LoggedNetworkNumber("/Intake/tuning/maxVelRadPerSec", DEFAULT_MAX_VEL);
	public LoggedNetworkNumber maxAccel =
			new LoggedNetworkNumber("/Intake/tuning/maxAccelRadPerSec2", DEFAULT_MAX_ACCEL);
	public LoggedNetworkNumber intakeUpDeg =
			new LoggedNetworkNumber("/Intake/tuning/intakeUpDeg", DEFAULT_INTAKE_UP_DEG);
	public LoggedNetworkNumber zeroGoalDeg =
			new LoggedNetworkNumber("/Intake/tuning/zeroGoalDeg", DEFAULT_ZERO_GOAL_DEG);
	public LoggedNetworkNumber intakeDownBiasVolts =
			new LoggedNetworkNumber("/Intake/tuning/intakeDownBiasVolts", DEFAULT_INTAKE_DOWN_BIAS_VOLTS);
	public LoggedNetworkNumber experimentSetpointDeg =
			new LoggedNetworkNumber("/Intake/tuning/experimentSetpointDeg", 0.0);
	public LoggedNetworkNumber fbDisableLoDeg =
			new LoggedNetworkNumber("/Intake/tuning/fbDisableLoDeg", DEFAULT_FB_DISABLE_LO_DEG);
	public LoggedNetworkNumber fbDisableHiDeg =
			new LoggedNetworkNumber("/Intake/tuning/fbDisableHiDeg", DEFAULT_FB_DISABLE_HI_DEG);
	public LoggedNetworkNumber fbDisableAboveDeg =
			new LoggedNetworkNumber("/Intake/tuning/fbDisableAboveDeg", DEFAULT_FB_DISABLE_ABOVE_DEG);

	public IntakeHW hw = new IntakeHW();
	public IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	public PIDController pivotPID = new PIDController(DEFAULT_kP, 0.0, DEFAULT_kD);
	public TrapezoidProfile pivotProfile = new TrapezoidProfile(new Constraints(DEFAULT_MAX_VEL, DEFAULT_MAX_ACCEL));

	public double targetPivotVoltage = 0;
	public double targetIntakeVoltage = 0;
	public Double targetPivotPositionRadians = null;
	public Double targetPivotExperimentRadians = null;
	public double targetPivotFeedforwardBiasVolts = 0;

	public State pivotSetpoint = new State(0, 0);

	public static boolean hasZeroed = false;

	private double lastMaxVel = DEFAULT_MAX_VEL;
	private double lastMaxAccel = DEFAULT_MAX_ACCEL;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Intake", inputs);

		pivotPID.setPID(kP.get(), 0.0, kD.get());

		double mv = maxVel.get();
		double ma = maxAccel.get();
		if (mv != lastMaxVel || ma != lastMaxAccel) {
			pivotProfile = new TrapezoidProfile(new Constraints(mv, ma));
			lastMaxVel = mv;
			lastMaxAccel = ma;
		}

		if (robot.isDisabled()) {
			targetPivotVoltage = 0;
			targetPivotPositionRadians = null;
			targetPivotExperimentRadians = null;
			targetPivotFeedforwardBiasVolts = 0;
			pivotSetpoint = new State(inputs.pivotPosRadians, 0);
			pivotPID.reset();

			if (inputs.pivotPosRadians < 0) {
				hw.zeroPivotForced();
				Logger.recordOutput("/Intake/autoZeroTriggered", true);
			} else {
				Logger.recordOutput("/Intake/autoZeroTriggered", false);
			}
		}
	}

	public double gravityFF() {
		return kG.get() * Math.cos(inputs.pivotPosRadians + Math.toRadians(gOffsetDeg.get()));
	}

	public boolean isFeedbackDisabled() {
		double posDeg = Math.toDegrees(inputs.pivotPosRadians);
		return (posDeg >= fbDisableLoDeg.get() && posDeg <= fbDisableHiDeg.get())
				|| posDeg > fbDisableAboveDeg.get();
	}

	public void actuate() {
		Logger.recordOutput("/Intake/targetIntakeVoltageVolts", targetIntakeVoltage);

		double pivotOutput;
		String mode;

		if (robot.isDisabled()) {
			pivotOutput = 0;
			mode = "disabled";
		} else if (targetPivotExperimentRadians != null) {
			double goal = targetPivotExperimentRadians;
			double posErr = goal - inputs.pivotPosRadians;
			double velErr = 0.0 - inputs.pivotVelRadiansPerSec;
			double pidOut = kP.get() * posErr + kD.get() * velErr;
			if (isFeedbackDisabled()) pidOut = 0;
			double ff = gravityFF();
			pivotOutput = pidOut + ff + targetPivotFeedforwardBiasVolts;
			pivotSetpoint = new State(inputs.pivotPosRadians, 0);

			Logger.recordOutput("/Intake/pivotGoalRad", goal);
			Logger.recordOutput("/Intake/pivotPidOutVolts", pidOut);
			Logger.recordOutput("/Intake/pivotFfVolts", ff);
			mode = "experiment";
		} else if (targetPivotPositionRadians != null) {
			double goal = targetPivotPositionRadians;
			pivotSetpoint = pivotProfile.calculate(LOOP_PERIOD, pivotSetpoint, new State(goal, 0));

			double posErr = pivotSetpoint.position - inputs.pivotPosRadians;
			double velErr = pivotSetpoint.velocity - inputs.pivotVelRadiansPerSec;
			double pidOut = kP.get() * posErr + kD.get() * velErr;
			if (isFeedbackDisabled()) pidOut = 0;
			double ff = gravityFF();
			double velFF = kV.get() * pivotSetpoint.velocity;
			pivotOutput = pidOut + ff + velFF + targetPivotFeedforwardBiasVolts;

			Logger.recordOutput("/Intake/pivotSetpointPosRad", pivotSetpoint.position);
			Logger.recordOutput("/Intake/pivotSetpointVelRadPerSec", pivotSetpoint.velocity);
			Logger.recordOutput("/Intake/pivotGoalRad", goal);
			Logger.recordOutput("/Intake/pivotPidOutVolts", pidOut);
			Logger.recordOutput("/Intake/pivotFfVolts", ff);
			Logger.recordOutput("/Intake/pivotVelFfVolts", velFF);
			mode = "profiled";
		} else {
			pivotOutput = targetPivotVoltage;
			pivotSetpoint = new State(inputs.pivotPosRadians, 0);
			pivotPID.reset();
			mode = "open-loop";
		}

		Logger.recordOutput("/Intake/pivotOutputVolts", pivotOutput);
		Logger.recordOutput("/Intake/pivotBiasVolts", targetPivotFeedforwardBiasVolts);
		Logger.recordOutput("/Intake/mode", mode);
		Logger.recordOutput("/Intake/hasZeroed", hasZeroed);

		hw.actuate(inputs, pivotOutput, targetIntakeVoltage);
	}
}
