package frc.robot.subsystem.drive;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
	private final StatusSignal<Angle> yaw = pigeon.getYaw();
	private final Queue<Double> yawTimestampQueue;
	private final Queue<Double> yawPositionQueue;
	private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
	private final StatusSignal<Angle> pitch = pigeon.getPitch();
	private final StatusSignal<Angle> roll = pigeon.getRoll();

	public GyroIOPigeon2() {
		if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
			tryUntilOk(
					5, () -> pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs, 0.25));
		} else {
			tryUntilOk(5, () -> pigeon.getConfigurator().apply(new Pigeon2Configuration(), 0.25));
		}
		tryUntilOk(5, () -> pigeon.getConfigurator().setYaw(0.0, 0.25));
		yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
		yawVelocity.setUpdateFrequency(50.0);
		pitch.setUpdateFrequency(50.0);
		roll.setUpdateFrequency(50.0);
		pigeon.optimizeBusUtilization();

		yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
		yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
		tryUntilOk(5, () -> pigeon.clearStickyFaults(0.25));
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected =
				BaseStatusSignal.refreshAll(yaw, yawVelocity, pitch, roll).equals(StatusCode.OK);
		inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
		inputs.pitchRadians = pitch.getValueAsDouble() * Math.PI / 180.0;
		inputs.rollRadians = roll.getValueAsDouble() * Math.PI / 180.0;

		inputs.odometryYawTimestamps =
				yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
		inputs.odometryYawPositions =
				yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
		yawTimestampQueue.clear();
		yawPositionQueue.clear();
	}
}
