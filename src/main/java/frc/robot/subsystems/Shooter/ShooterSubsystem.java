package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

        public TalonFX flywheelMotor;
        public TalonFX flywheelFollowerMotor;
    
        public TalonFX elevatorMotor;
    public ShooterSubsystem() {
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
        
    }

    public void setShooterSpeed(double speed) {
        flywheelMotor.setControl(new DutyCycleOut(speed));
    }
    public void setElevatorSpeed(double speed) {
        elevatorMotor.setControl(new DutyCycleOut(speed));
    }
    public void stopShooter() {
        flywheelMotor.setControl(new DutyCycleOut(0.0));
    }

   
}
