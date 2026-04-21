package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControls implements Controls {
	public static CommandXboxController xbox = new CommandXboxController(0); // FIXME. controller port

	public Trigger align() {
		return xbox.rightBumper();
	}

	public Trigger shoot() {
		return xbox.rightTrigger();
	}

	public Trigger extendOut() {
		return xbox.x();
	}

	public Trigger extendIn() {
		return xbox.b();
	}

	public Trigger xLock() {
		return xbox.leftBumper();
	}

	public Trigger intakeReverse() {
		return xbox.y();
	}

	public Trigger invertToggle() {
		return xbox.leftStick();
	}

	public Trigger sysidQuasiForward() {
		return xbox.back().and(xbox.y());
	}

	public Trigger sysidQuasiBackward() {
		return xbox.back().and(xbox.x());
	}

	public Trigger sysidDynaForward() {
		return xbox.start().and(xbox.y());
	}

	public Trigger sysidDynaBackward() {
		return xbox.start().and(xbox.x());
	}

	public Trigger pivotNudgeUp() {
		return new Trigger(() -> false);
	}

	public Trigger pivotNudgeDown() {
		return new Trigger(() -> false);
	}

	public Trigger shooterStop() {
		return new Trigger(() -> false);
	}

	public double leftX() {
		return -xbox.getLeftX();
	}

	public double leftY() {
		return -xbox.getLeftY();
	}

	public double rightX() {
		return -xbox.getRightX();
	}

	public double rightY() {
		return -xbox.getRightY();
	}

	public double thirdX() {
		return 0;
	}

	public double thirdY() {
		return 0;
	}
}
