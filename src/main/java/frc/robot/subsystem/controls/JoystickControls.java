package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickControls implements Controls {
	public static CommandJoystick left = new CommandJoystick(1);
	public static CommandJoystick right = new CommandJoystick(2);
	public static CommandJoystick ds = new CommandJoystick(5);

	public static boolean inverted = false;

	public Trigger align() {
		return right.button(2);
	}

	public Trigger shoot() {
		return right.button(1);
	}

	public Trigger extendOut() {
		return ds.button(12);
	}

	public Trigger extendIn() {
		return ds.button(11);
	}

	public Trigger xLock() {
		return left.button(2);
	}

	public Trigger intakeReverse() {
		return ds.button(6);
	}

	public Trigger sysidQuasiForward() {
		return left.button(1);
	}

	public Trigger sysidQuasiBackward() {
		return left.button(2);
	}

	public Trigger sysidDynaForward() {
		return left.button(3);
	}

	public Trigger sysidDynaBackward() {
		return left.button(4);
	}

	public Trigger pivotNudgeUp() {
		return ds.button(9);
	}

	public Trigger pivotNudgeDown() {
		return ds.button(10);
	}

	public Trigger invertToggle() {
		return ds.button(2);
	}

	public Trigger shooterStop() {
		return ds.button(6);
	}

	public double leftX() {
		double mul = inverted ? -1 : 1;
		return mul * left.getX();
	}

	public double leftY() {
		double mul = inverted ? -1 : 1;
		return mul * left.getY();
	}

	public double rightX() {
		return -right.getX();
	}

	public double rightY() {
		return -right.getY();
	}

	public double thirdX() {
		return ds.getX();
	}

	public double thirdY() {
		return ds.getY();
	}
}
