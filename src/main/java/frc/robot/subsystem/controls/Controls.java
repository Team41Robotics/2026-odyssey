package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controls {
	Trigger align();

	Trigger shoot();

	Trigger extendOut();

	Trigger extendIn();

	Trigger xLock();

	Trigger intakeReverse();

	Trigger invertToggle();

	Trigger sysidQuasiForward();

	Trigger sysidQuasiBackward();

	Trigger sysidDynaForward();

	Trigger sysidDynaBackward();

	Trigger pivotNudgeUp();

	Trigger pivotNudgeDown();

	double leftX();

	double leftY();

	double rightX();

	double rightY();

	double thirdX();

	double thirdY();
}
