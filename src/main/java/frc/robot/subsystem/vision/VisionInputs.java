package frc.robot.subsystem.vision;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class VisionInputs {
	public boolean isConnected;

	public byte[] data = new byte[0];

	public double[] cameraMatrix = new double[0];
	public double[] distCoeffs = new double[0];
}
