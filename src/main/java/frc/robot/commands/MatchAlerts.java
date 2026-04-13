package frc.robot.commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MatchAlerts extends Command {
	public static final double STALE_Sec = 0.300;
	public static final double MIN_LOG_TS_Sec = 0.050;

	// CAN IDs in this project (max=62), array covers 0-62
	static final int ARRAY_SIZE = 63;

	public static long[] prevHash = new long[ARRAY_SIZE];
	public static double[] prevTsSec = new double[ARRAY_SIZE];
	public static double[] lastUpdateTimeSec = new double[ARRAY_SIZE];
	public static double[] downStartTimeSec = new double[ARRAY_SIZE];
	public static double[] totalDownSec = new double[ARRAY_SIZE];
	public static double[] totalEverDownSec = new double[ARRAY_SIZE];
	public static boolean[] hasDisconnectedOnce = new boolean[ARRAY_SIZE];

	static {
		for (int i = 0; i < ARRAY_SIZE; i++) {
			prevHash[i] = 0;
			prevTsSec[i] = Double.NaN;
			lastUpdateTimeSec[i] = 0;
			downStartTimeSec[i] = Double.NaN;
			totalDownSec[i] = 0;
			totalEverDownSec[i] = 0;
		}
	}

	// FL module (index 0): drive=6, steer=5, encoder=16
	public static Alert swerveFL_Drive = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveFL_Steer = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveFL_Encoder = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveFL_DriveError = new Alert("MatchAlerts", "", AlertType.kError);
	public static Alert swerveFL_SteerError = new Alert("MatchAlerts", "", AlertType.kError);
	public static Alert swerveFL_EncoderError = new Alert("MatchAlerts", "", AlertType.kError);

	// FR module (index 1): drive=12, steer=11, encoder=17
	public static Alert swerveFR_Drive = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveFR_Steer = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveFR_Encoder = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveFR_DriveError = new Alert("MatchAlerts", "", AlertType.kError);
	public static Alert swerveFR_SteerError = new Alert("MatchAlerts", "", AlertType.kError);
	public static Alert swerveFR_EncoderError = new Alert("MatchAlerts", "", AlertType.kError);

	// BL module (index 2): drive=8, steer=7, encoder=18
	public static Alert swerveBL_Drive = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveBL_Steer = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveBL_Encoder = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveBL_DriveError = new Alert("MatchAlerts", "", AlertType.kError);
	public static Alert swerveBL_SteerError = new Alert("MatchAlerts", "", AlertType.kError);
	public static Alert swerveBL_EncoderError = new Alert("MatchAlerts", "", AlertType.kError);

	// BR module (index 3): drive=1, steer=9, encoder=15
	public static Alert swerveBR_Drive = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveBR_Steer = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveBR_Encoder = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert swerveBR_DriveError = new Alert("MatchAlerts", "", AlertType.kError);
	public static Alert swerveBR_SteerError = new Alert("MatchAlerts", "", AlertType.kError);
	public static Alert swerveBR_EncoderError = new Alert("MatchAlerts", "", AlertType.kError);

	// Gyro (ID 2)
	public static Alert imuAlert = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert imuError = new Alert("MatchAlerts", "", AlertType.kError);

	// Intake pivot (ID 21), roller (ID 31)
	public static Alert intakePivotAlert = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert intakePivotError = new Alert("MatchAlerts", "", AlertType.kError);
	public static Alert intakeRollerAlert = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert intakeRollerError = new Alert("MatchAlerts", "", AlertType.kError);

	// Indexer roller (ID 60), indexer follower (ID 32)
	public static Alert indexerRollerAlert = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert indexerRollerError = new Alert("MatchAlerts", "", AlertType.kError);
	public static Alert indexerFollowerAlert = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert indexerFollowerError = new Alert("MatchAlerts", "", AlertType.kError);

	// Shooter flywheel leader (ID 62)
	public static Alert flywheelAlert = new Alert("MatchAlerts", "", AlertType.kWarning);
	public static Alert flywheelError = new Alert("MatchAlerts", "", AlertType.kError);

	/**
	 * Unified device disconnect checker.
	 *
	 * <p>A device is considered "up" when either:
	 *
	 * <ul>
	 *   <li>any monitored value changes (state fingerprint changes), or
	 *   <li>any CTRE timestamp increases (phoenix StatusSignal timestamp changes)
	 * </ul>
	 *
	 * <p>When no change is observed for {@link #STALE_Sec}, the warning alert is set. Once a device
	 * has ever been down, the error alert is latched for the rest of the program run.
	 */
	static void check(Alert warnAlert, Alert errorAlert, String name, int canId, double[] values, double[] tsSec) {
		double now = Timer.getTimestamp();

		long h = 0xcbf29ce484222325L; // FNV-1a 64-bit offset
		if (values != null) {
			for (double v : values) {
				long bits = Double.doubleToLongBits(v);
				h ^= bits;
				h *= 0x100000001b3L;
			}
		}
		double bestTs = Double.NaN;
		if (tsSec != null) {
			for (double t : tsSec) {
				if (Double.isFinite(t) && t > MIN_LOG_TS_Sec) {
					if (!Double.isFinite(bestTs) || t > bestTs) bestTs = t;
				}
			}
			for (double t : tsSec) {
				long bits = Double.doubleToLongBits(t);
				h ^= bits;
				h *= 0x100000001b3L;
			}
		}

		boolean updated = false;
		if (h != prevHash[canId]) {
			prevHash[canId] = h;
			updated = true;
		}
		if (Double.isFinite(bestTs) && (Double.isNaN(prevTsSec[canId]) || bestTs != prevTsSec[canId])) {
			prevTsSec[canId] = bestTs;
			updated = true;
		}

		if (updated) {
			lastUpdateTimeSec[canId] = now;
		}

		boolean downNow = (now - lastUpdateTimeSec[canId]) > STALE_Sec;
		if (downNow) {
			hasDisconnectedOnce[canId] = true;
			if (!Double.isFinite(downStartTimeSec[canId])) {
				downStartTimeSec[canId] = now;
			}
		} else {
			if (Double.isFinite(downStartTimeSec[canId])) {
				double dt = now - downStartTimeSec[canId];
				totalDownSec[canId] += dt;
				totalEverDownSec[canId] += dt;
				downStartTimeSec[canId] = Double.NaN;
			}
			warnAlert.set(false);
		}

		if (downNow) {
			double currentDown = now - downStartTimeSec[canId];
			warnAlert.setText(
					name + " is DOWN" + String.format(" (down %.2fs total)", totalDownSec[canId] + currentDown));
			warnAlert.set(true);
		}

		if (hasDisconnectedOnce[canId]) {
			double everDown = totalEverDownSec[canId];
			if (downNow) everDown += (now - downStartTimeSec[canId]);
			errorAlert.setText(name + " has disconnected" + String.format(" (down %.2fs total)", everDown));
			errorAlert.set(true);
		} else {
			errorAlert.set(false);
		}
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	@Override
	public void execute() {
		// FL module (index 0): drive=6, steer=5, encoder=16
		check(
				swerveFL_Drive,
				swerveFL_DriveError,
				"Swerve FL Drive (6)",
				6,
				new double[] {
					drive.modules[0].inputs.driveConnected ? 1.0 : 0.0,
					drive.modules[0].inputs.driveAppliedVolts,
					drive.modules[0].inputs.driveCurrentAmps,
					drive.modules[0].inputs.driveVelocityRadPerSec,
					drive.modules[0].inputs.driveSupplyVoltage
				},
				drive.modules[0].inputs.odometryTimestamps);
		check(
				swerveFL_Steer,
				swerveFL_SteerError,
				"Swerve FL Steer (5)",
				5,
				new double[] {
					drive.modules[0].inputs.turnConnected ? 1.0 : 0.0,
					drive.modules[0].inputs.turnAppliedVolts,
					drive.modules[0].inputs.turnCurrentAmps,
					drive.modules[0].inputs.turnVelocityRadPerSec,
					drive.modules[0].inputs.turnSupplyVoltage
				},
				null);
		check(
				swerveFL_Encoder,
				swerveFL_EncoderError,
				"Swerve FL Encoder (16)",
				16,
				new double[] {
					drive.modules[0].inputs.turnEncoderConnected ? 1.0 : 0.0,
					drive.modules[0].inputs.turnAbsolutePosition.getRadians()
				},
				null);

		// FR module (index 1): drive=12, steer=11, encoder=17
		check(
				swerveFR_Drive,
				swerveFR_DriveError,
				"Swerve FR Drive (12)",
				12,
				new double[] {
					drive.modules[1].inputs.driveConnected ? 1.0 : 0.0,
					drive.modules[1].inputs.driveAppliedVolts,
					drive.modules[1].inputs.driveCurrentAmps,
					drive.modules[1].inputs.driveVelocityRadPerSec,
					drive.modules[1].inputs.driveSupplyVoltage
				},
				drive.modules[1].inputs.odometryTimestamps);
		check(
				swerveFR_Steer,
				swerveFR_SteerError,
				"Swerve FR Steer (11)",
				11,
				new double[] {
					drive.modules[1].inputs.turnConnected ? 1.0 : 0.0,
					drive.modules[1].inputs.turnAppliedVolts,
					drive.modules[1].inputs.turnCurrentAmps,
					drive.modules[1].inputs.turnVelocityRadPerSec,
					drive.modules[1].inputs.turnSupplyVoltage
				},
				null);
		check(
				swerveFR_Encoder,
				swerveFR_EncoderError,
				"Swerve FR Encoder (17)",
				17,
				new double[] {
					drive.modules[1].inputs.turnEncoderConnected ? 1.0 : 0.0,
					drive.modules[1].inputs.turnAbsolutePosition.getRadians()
				},
				null);

		// BL module (index 2): drive=8, steer=7, encoder=18
		check(
				swerveBL_Drive,
				swerveBL_DriveError,
				"Swerve BL Drive (8)",
				8,
				new double[] {
					drive.modules[2].inputs.driveConnected ? 1.0 : 0.0,
					drive.modules[2].inputs.driveAppliedVolts,
					drive.modules[2].inputs.driveCurrentAmps,
					drive.modules[2].inputs.driveVelocityRadPerSec,
					drive.modules[2].inputs.driveSupplyVoltage
				},
				drive.modules[2].inputs.odometryTimestamps);
		check(
				swerveBL_Steer,
				swerveBL_SteerError,
				"Swerve BL Steer (7)",
				7,
				new double[] {
					drive.modules[2].inputs.turnConnected ? 1.0 : 0.0,
					drive.modules[2].inputs.turnAppliedVolts,
					drive.modules[2].inputs.turnCurrentAmps,
					drive.modules[2].inputs.turnVelocityRadPerSec,
					drive.modules[2].inputs.turnSupplyVoltage
				},
				null);
		check(
				swerveBL_Encoder,
				swerveBL_EncoderError,
				"Swerve BL Encoder (18)",
				18,
				new double[] {
					drive.modules[2].inputs.turnEncoderConnected ? 1.0 : 0.0,
					drive.modules[2].inputs.turnAbsolutePosition.getRadians()
				},
				null);

		// BR module (index 3): drive=1, steer=9, encoder=15
		check(
				swerveBR_Drive,
				swerveBR_DriveError,
				"Swerve BR Drive (1)",
				1,
				new double[] {
					drive.modules[3].inputs.driveConnected ? 1.0 : 0.0,
					drive.modules[3].inputs.driveAppliedVolts,
					drive.modules[3].inputs.driveCurrentAmps,
					drive.modules[3].inputs.driveVelocityRadPerSec,
					drive.modules[3].inputs.driveSupplyVoltage
				},
				drive.modules[3].inputs.odometryTimestamps);
		check(
				swerveBR_Steer,
				swerveBR_SteerError,
				"Swerve BR Steer (9)",
				9,
				new double[] {
					drive.modules[3].inputs.turnConnected ? 1.0 : 0.0,
					drive.modules[3].inputs.turnAppliedVolts,
					drive.modules[3].inputs.turnCurrentAmps,
					drive.modules[3].inputs.turnVelocityRadPerSec,
					drive.modules[3].inputs.turnSupplyVoltage
				},
				null);
		check(
				swerveBR_Encoder,
				swerveBR_EncoderError,
				"Swerve BR Encoder (15)",
				15,
				new double[] {
					drive.modules[3].inputs.turnEncoderConnected ? 1.0 : 0.0,
					drive.modules[3].inputs.turnAbsolutePosition.getRadians()
				},
				null);

		// Gyro Pigeon2 (ID 2)
		check(
				imuAlert,
				imuError,
				"IMU Pigeon2 (2)",
				2,
				new double[] {
					drive.gyroInputs.connected ? 1.0 : 0.0,
					drive.gyroInputs.yawVelocityRadPerSec,
					drive.gyroInputs.pitchRadians,
					drive.gyroInputs.rollRadians
				},
				drive.gyroInputs.odometryYawTimestamps);

		// Intake pivot (ID 21)
		check(
				intakePivotAlert,
				intakePivotError,
				"Intake Pivot (21)",
				21,
				new double[] {
					intake.inputs.pivotBusVoltageVolts,
					intake.inputs.pivotVoltageVolts,
					intake.inputs.pivotCurrentAmps,
					intake.inputs.pivotPosRadians,
					intake.inputs.pivotVelRadiansPerSec
				},
				null);

		// Intake roller (ID 31)
		check(
				intakeRollerAlert,
				intakeRollerError,
				"Intake Roller (31)",
				31,
				new double[] {
					intake.inputs.rollerBusVoltageVolts,
					intake.inputs.intakeVoltageVolts,
					intake.inputs.intakeCurrentAmps,
					intake.inputs.rollerVelocityRPS
				},
				null);

		// Indexer roller (ID 60)
		check(
				indexerRollerAlert,
				indexerRollerError,
				"Indexer Roller (60)",
				60,
				new double[] {
					indexer.inputs.rollerBusVoltageVolts,
					indexer.inputs.rollerVoltageVolts,
					indexer.inputs.rollerCurrentAmps,
					indexer.inputs.rollerVelocityRPM
				},
				null);

		// Indexer follower (ID 32)
		check(
				indexerFollowerAlert,
				indexerFollowerError,
				"Indexer Follower (32)",
				32,
				new double[] {
					indexer.inputs.indexerBusVoltageVolts,
					indexer.inputs.indexerVoltageVolts,
					indexer.inputs.indexerCurrentAmps,
					indexer.inputs.indexerVelocityRPM
				},
				null);

		// Shooter flywheel leader (ID 62)
		check(
				flywheelAlert,
				flywheelError,
				"Shooter Flywheel (62)",
				62,
				new double[] {
					shooter.inputs.flywheelBusVoltageVolts,
					shooter.inputs.flywheelVoltageVolts,
					shooter.inputs.flywheelCurrentAmps,
					shooter.inputs.flywheelVelocityRPM
				},
				new double[] {shooter.inputs.flywheelTsSec});
	}
}
