package frc.robot.subsystem.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
	public static final double kP = 15.0;
	public static final double kD = 0.04;
	public static final double kG = 1.5;
	public static final double kV = 1.5;
	public static final double G_OFFSET_DEG = 15.0;
	public static final double MAX_VEL = 3.0;
	public static final double MAX_ACCEL = 14.0;
	public static final double INTAKE_UP_DEG = 65.0;
	public static final double ZERO_GOAL_DEG = 20.0;
	public static final double INTAKE_DOWN_BIAS_VOLTS = 1.0;
	public static final double FB_DISABLE_LO_DEG = 10.0;
	public static final double FB_DISABLE_HI_DEG = 45.0;
	public static final double FB_DISABLE_ABOVE_DEG = 900.0;

	public IntakeHW hw = new IntakeHW();
	public IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	public PIDController pivotPID = new PIDController(kP, 0.0, kD);
	public TrapezoidProfile pivotProfile = new TrapezoidProfile(new Constraints(MAX_VEL, MAX_ACCEL));

	public double targetPivotVoltage = 0;
	public double targetIntakeVoltage = 0;
	public Double targetPivotPositionRadians = null;
	public double targetPivotFeedforwardBiasVolts = 0;

	public State pivotSetpoint = new State(0, 0);

	public static boolean hasZeroed = false;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Intake", inputs);

		if (robot.isDisabled()) {
			targetPivotVoltage = 0;
			targetPivotPositionRadians = null;
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
		return kG * Math.cos(inputs.pivotPosRadians + Math.toRadians(G_OFFSET_DEG));
	}

	public boolean isFeedbackDisabled() {
		double posDeg = Math.toDegrees(inputs.pivotPosRadians);
		return (posDeg >= FB_DISABLE_LO_DEG && posDeg <= FB_DISABLE_HI_DEG) || posDeg > FB_DISABLE_ABOVE_DEG;
	}

	public void actuate() {
		Logger.recordOutput("/Intake/targetIntakeVoltageVolts", targetIntakeVoltage);

		double pivotOutput;
		String mode;

		if (robot.isDisabled()) {
			pivotOutput = 0;
			mode = "disabled";
		} else if (targetPivotPositionRadians != null) {
			double goal = targetPivotPositionRadians;
			pivotSetpoint = pivotProfile.calculate(LOOP_PERIOD, pivotSetpoint, new State(goal, 0));

			double posErr = pivotSetpoint.position - inputs.pivotPosRadians;
			double velErr = pivotSetpoint.velocity - inputs.pivotVelRadiansPerSec;
			double pidOut = kP * posErr + kD * velErr;
			if (isFeedbackDisabled()) pidOut = 0;
			double ff = gravityFF();
			double velFF = kV * pivotSetpoint.velocity;
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
