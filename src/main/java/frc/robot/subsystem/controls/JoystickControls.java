package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickControls implements Controls {
	public static CommandJoystick left = new CommandJoystick(1); // FIXME: controller ports
	public static CommandJoystick right = new CommandJoystick(2);

	public Trigger shootTrigger() {
		return right.trigger();
	}

	public Trigger elevatorUp() {
		return right.button(3);
	}

	public Trigger intakeTrigger() {
		return left.trigger();
	}

	public Trigger fullShootBumper() {
		return right.button(2);
	}

	public Trigger feederButton() {
		return left.button(2);
	}

	public Trigger extendOut() {
		return left.button(3);
	}

	public Trigger extendIn() {
		return left.button(4);
	}

	public Trigger xLock() {
		return left.button(5);
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
		return -left.getY();
	}

	public double rightX() {
		return right.getX();
	}

	public double rightY() {
		return -right.getY();
	}
}
