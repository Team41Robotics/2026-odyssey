package frc.robot.subsystem.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
	static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
	public static final double DRIVE_BASE_RADIUS = Math.max(
			Math.max(
					Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
					Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
			Math.max(
					Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
					Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

	static final Lock odometryLock = new ReentrantLock();

	private final GyroIO gyroIO;
	public final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	public final Module[] modules = new Module[4]; // FL, FR, BL, BR
	private final SysIdRoutine sysId;
	private final Alert gyroDisconnectedAlert =
			new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

	private boolean brakeMode = false;

	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
	private Rotation2d rawGyroRotation = Rotation2d.kZero;
	private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
		new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
	};
	private SwerveDrivePoseEstimator poseEstimator =
			new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

	public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
		this.gyroIO = gyroIO;
		modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
		modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
		modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
		modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

		PhoenixOdometryThread.getInstance().start();

		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(
						null, null, null, state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
				new SysIdRoutine.Mechanism(voltage -> runCharacterization(voltage.in(Volts)), null, this));
	}

	@Override
	public void periodic() {
		odometryLock.lock();
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);
		for (var module : modules) {
			module.periodic();
		}
		odometryLock.unlock();

		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
		}

		// Process all high-frequency odometry samples accumulated since last cycle
		double[] sampleTimestamps = modules[0].getOdometryTimestamps();
		int sampleCount = sampleTimestamps.length;
		for (int i = 0; i < sampleCount; i++) {
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
			for (int m = 0; m < 4; m++) {
				modulePositions[m] = modules[m].getOdometryPositions()[i];
				moduleDeltas[m] = new SwerveModulePosition(
						modulePositions[m].distanceMeters - lastModulePositions[m].distanceMeters,
						modulePositions[m].angle);
				lastModulePositions[m] = modulePositions[m];
			}
			if (gyroInputs.connected) {
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}
			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
		}

		gyroDisconnectedAlert.set(!gyroInputs.connected);
		Logger.recordOutput("Odometry/Robot", getPose());
		Logger.recordOutput("Odometry/RawGyro", rawGyroRotation);
		Logger.recordOutput("SwerveStates/Measured", getModuleStates());
		Logger.recordOutput("SwerveChassisSpeeds/Measured", getChassisSpeeds());
		Logger.recordOutput("Odometry/SampleCount", sampleCount);
		Logger.recordOutput("Odometry/GyroConnected", gyroInputs.connected);
	}

	/** Commands robot-relative chassis speeds (discretized). */
	public void runVelocity(ChassisSpeeds speeds) {
		Logger.recordOutput("SwerveChassisSpeeds/Target", speeds);
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
		SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
		Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
		for (int i = 0; i < 4; i++) {
			modules[i].runSetpoint(setpointStates[i]);
		}
		Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
	}

	public void runCharacterization(double output) {
		for (int i = 0; i < 4; i++) {
			modules[i].runCharacterization(output);
		}
	}

	public void stop() {
		runVelocity(new ChassisSpeeds());
	}

	/** Points all modules inward to resist pushing (X-lock). */
	public void stopWithX() {
		Rotation2d[] headings = new Rotation2d[4];
		for (int i = 0; i < 4; i++) {
			headings[i] = getModuleTranslations()[i].getAngle();
		}
		kinematics.resetHeadings(headings);
		stop();
	}

	public void setBrakeMode(boolean brake) {
		if (brake == brakeMode) return;
		brakeMode = brake;
		for (var module : modules) {
			module.setBrakeMode(brake);
		}
	}

	/** Resets the odometry heading to zero while preserving the current field position. */
	public void zeroHeading() {
		setPose(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
	}

	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			positions[i] = modules[i].getPosition();
		}
		return positions;
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Rotation2d getRotation() {
		return getPose().getRotation();
	}

	public void setPose(Pose2d pose) {
		poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
	}

	public void addVisionMeasurement(
			Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
		poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
	}

	public double getMaxLinearSpeedMetersPerSec() {
		return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
	}

	public double getMaxAngularSpeedRadPerSec() {
		return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
	}

	public static Translation2d[] getModuleTranslations() {
		return new Translation2d[] {
			new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
			new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
			new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
			new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
		};
	}
}
