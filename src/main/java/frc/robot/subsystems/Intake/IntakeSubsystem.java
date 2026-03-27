package frc.robot.subsystems.Intake;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {
        public TalonFX intakeMotor;
        public TalonFX extensionMotor;
        public TalonFX extensionMotorFollower;
        public TalonFX feederMotor;
    public IntakeSubsystem(){
        intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID); 
        feederMotor = new TalonFX(Constants.IntakeConstants.FEEDER_MOTOR_ID);
        extensionMotor = new TalonFX(IntakeConstants.EXTENSION_MOTOR_ID);
        extensionMotorFollower = new TalonFX(IntakeConstants.EXTENSION_FOLLOWER_MOTOR_ID);

        extensionMotorFollower.setControl(new Follower(IntakeConstants.EXTENSION_MOTOR_ID, MotorAlignmentValue.Opposed) );

    }
    public void setIntakeSpeed(double speed) {
        intakeMotor.setControl(new DutyCycleOut(speed));
    }
    public void stopIntake() {
        intakeMotor.setControl(new DutyCycleOut(0.0));
    }
    public void setExtensionSpeed(double speed) {
        extensionMotor.setControl(new DutyCycleOut(speed));
    }
    public void setFeederSpeed(double speed){
        feederMotor.setControl(new DutyCycleOut(speed));
    }
    public void extendOut(){
        extensionMotor.setControl(new DutyCycleOut(.4));
        //extensionMotorFollower.setControl(new DutyCycleOut(-.4));

    }
    public void extendIn(){
        extensionMotor.setControl(new DutyCycleOut(-.4));
        //extensionMotorFollower.setControl(new DutyCycleOut(-.4));

    }
        public void extendStop(){
        extensionMotor.setControl(new DutyCycleOut(0));
        // extensionMotorFollower.setControl(new DutyCycleOut(0));

    }
}