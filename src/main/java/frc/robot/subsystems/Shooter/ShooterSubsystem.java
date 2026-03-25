package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.InterpTables;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ShooterSubsystem extends SubsystemBase {

        public final CommandSwerveDrivetrain drive;
        public TalonFX flywheelMotor;
        public TalonFX flywheelFollowerMotor;
        public InterpTables tables = new InterpTables();
    
        public TalonFX elevatorMotor;
        public double tuningVel = 100.0; // RPS, adjust as necessary for tuning
        public double testVel = 50.0; // RPS, adjust as necessary for testing
    public ShooterSubsystem(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID); // Replace with actual CAN ID
        flywheelFollowerMotor = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_MOTOR_ID); // Replace with actual CAN ID
        elevatorMotor = new TalonFX(ShooterConstants.ELEVATOR_MOTOR_ID);
        // Configure the follower motor to follow the main motor
        flywheelFollowerMotor.setControl(new Follower(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed) );


        // set supply current limits on shooter motors
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 60.0; // Set supply current limit to 40Ai
                config.CurrentLimits.StatorCurrentLimit = 60.0; // Set supply current limit to 40Ai

        config.CurrentLimits.SupplyCurrentLimitEnable = true; // Enable supply current limit    
        flywheelMotor.getConfigurator().apply(config);
        flywheelFollowerMotor.getConfigurator().apply(config);
        
        // config lead motors pid slots for velocity control
        TalonFXConfiguration pidConfig = new TalonFXConfiguration();
        pidConfig.Slot0.kS = 0.00; // Static Feedforward. Increase until motor begins moving the slightest bit when at above 0 vel setpoint (step around 0.05)
        pidConfig.Slot0.kV = 0.0; // Velocity Feedforward. Increase until you reach the max velocity of the flywheel at tuning vel = 100% output. Then repeat process at tuningVel = 20% output and make sure kV is still correct. Adjust kV as necessary.

        pidConfig.Slot0.kP = 0.0; // Proportional gain
        pidConfig.Slot0.kI = 0.000; // Integral gain
        pidConfig.Slot0.kD = 0.0; // Derivative gain
        //once you have it tuned to a good set, test it out by setting tuning vel to differenbt velocities, then its time to begin getting the interp setting. Uncomment lines 
        flywheelMotor.getConfigurator().apply(pidConfig);



        //to tune shooter table, uncomment lines 134-145 in robotContainer, and use it to test shots at different distances, recording the optimal flywheel velocity for each distance. Press y to print out a line of code, put tat exact line of code into the InterpTables constructor, and repeat until you have a full table. Then you can use the getOptimalFlywheelSpeed method to get the optimal flywheel speed for any distance within the range of your table!
    }
    @Override
    public void periodic() {
        //logTuning();
        //flywheelMotor.setControl( new VelocityDutyCycle((tuningVel)));
    }

    public double getOptimalFlywheelSpeed(double distanceToHub) {
        return tables.flywheelSpeed.get(distanceToHub);
    }

    public Command shootCommand(double distanceToHub) {
        return this.run(() -> {
            double targetVel = tables.flywheelSpeed.get(distanceToHub);
            setShooterSpeed(targetVel); // Convert RPS to percentage output
        });
    }
    public void printTableEntry(){
        double velocity = flywheelMotor.getVelocity().getValueAsDouble();
        System.out.println("flywheelSpeed.put(" + drive.getHubDistance() + ", " + velocity + ");");
    }
    public void logTuning(){
        double velocity = flywheelMotor.getVelocity().getValueAsDouble();
        double output = tuningVel;
        System.out.println("Flywheel Velocity: " + velocity + " RPS, Target Vel: " + output);
    }
    public void setShooterSpeed(double speed) {
        flywheelMotor.setControl(new VelocityDutyCycle(speed));
    }
    public void setElevatorSpeed(double speed) {
        elevatorMotor.setControl(new DutyCycleOut(speed));
    }
    public void stopShooter() {
        flywheelMotor.setControl(new DutyCycleOut(0.0));
    }

   
}
