package frc.robot.commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class PreMatchCheck extends Command {
	public static boolean allPassed = false;

	public static Alert batteryOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert batteryFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert imuConnOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert imuConnFail = new Alert("PreMatch", "", AlertType.kError);

	// 4 modules x 2 (drive + steer) = 8
	public static Alert[] swerveOk = new Alert[8];
	public static Alert[] swerveFail = new Alert[8];

	// Up to 2 cameras (current config has 1)
	public static Alert[] visionOk = new Alert[2];
	public static Alert[] visionFail = new Alert[2];

	public static Alert intakePivotOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert intakePivotFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert intakeRollerOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert intakeRollerFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert indexerRollerOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert indexerRollerFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert indexerFollowerOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert indexerFollowerFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert flywheelOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert flywheelFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert summaryOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert summaryFail = new Alert("PreMatch", "", AlertType.kError);

	static {
		for (int i = 0; i < 8; i++) {
			swerveOk[i] = new Alert("PreMatch", "", AlertType.kInfo);
			swerveFail[i] = new Alert("PreMatch", "", AlertType.kError);
		}
		for (int i = 0; i < 2; i++) {
			visionOk[i] = new Alert("PreMatch", "", AlertType.kInfo);
			visionFail[i] = new Alert("PreMatch", "", AlertType.kError);
		}
	}

	static boolean check(Alert ok, Alert fail, String name, boolean passed) {
		ok.setText(name + " OK");
		fail.setText(name + " FAIL");
		ok.set(passed);
		fail.set(!passed);
		return passed;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	@Override
	public boolean isFinished() {
		return DriverStation.isEnabled();
	}

	@Override
	public void execute() {
		allPassed = true;

		double busV = drive.modules[0].inputs.driveSupplyVoltageVolts;
		allPassed &= check(batteryOk, batteryFail, "Battery " + String.format("%.1fV", busV), busV > 11.5);

		allPassed &= check(imuConnOk, imuConnFail, "IMU Connected", drive.gyroInputs.connected);

		String[] moduleNames = {"FL", "FR", "BL", "BR"};
		for (int i = 0; i < 4; i++) {
			allPassed &= check(
					swerveOk[i * 2],
					swerveFail[i * 2],
					"Swerve " + moduleNames[i] + " Drive",
					drive.modules[i].inputs.driveConnected);
			allPassed &= check(
					swerveOk[i * 2 + 1],
					swerveFail[i * 2 + 1],
					"Swerve " + moduleNames[i] + " Steer",
					drive.modules[i].inputs.turnConnected);
		}

		int nCams = Math.min(vision.nCams, visionOk.length);
		for (int i = 0; i < nCams; i++) {
			allPassed &=
					check(visionOk[i], visionFail[i], "Vision " + vision.cameras[i].name, vision.inputs[i].isConnected);
		}

		allPassed &= check(intakePivotOk, intakePivotFail, "Intake Pivot", intake.inputs.pivotBusVoltageVolts > 0);
		allPassed &= check(intakeRollerOk, intakeRollerFail, "Intake Roller", intake.inputs.rollerBusVoltageVolts > 0);

		allPassed &=
				check(indexerRollerOk, indexerRollerFail, "Indexer Roller", indexer.inputs.rollerBusVoltageVolts > 0);
		allPassed &= check(
				indexerFollowerOk, indexerFollowerFail, "Indexer Follower", indexer.inputs.indexerBusVoltageVolts > 0);

		allPassed &= check(flywheelOk, flywheelFail, "Shooter Flywheel", shooter.inputs.flywheelBusVoltageVolts > 0);

		check(summaryOk, summaryFail, "ALL SYSTEMS", allPassed);
	}
}
