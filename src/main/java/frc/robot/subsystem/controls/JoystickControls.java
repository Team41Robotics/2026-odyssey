package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickControls implements Controls {
	public static CommandJoystick left = new CommandJoystick(1);
	public static CommandJoystick right = new CommandJoystick(2);
	public static CommandJoystick ds = new CommandJoystick(5);

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

	public Trigger sysidQuasiForward() {
		return left.button(11).and(right.button(11));
	}

	public Trigger sysidQuasiBackward() {
		return left.button(11).and(right.button(10));
	}

	public Trigger sysidDynaForward() {
		return left.button(10).and(right.button(11));
	}

	public Trigger sysidDynaBackward() {
		return left.button(10).and(right.button(10));
	}

	public double leftX() {
		return left.getX();
	}

	public double leftY() {
		return left.getY();
	}

	public double rightX() {
		return -right.getX();
	}

	public double rightY() {
		return -right.getY();
	}
}
