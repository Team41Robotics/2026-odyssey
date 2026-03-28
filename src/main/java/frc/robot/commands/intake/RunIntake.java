package frc.robot.commands.intake;

import static frc.robot.RobotContainer.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.intake.IntakeHW;

public class RunIntake extends Command{
        public RunIntake(){
                addRequirements(intake);
        }
        @Override
        public void execute(){
                intake.targetIntakeVoltage = IntakeHW.INTAKE_VOLTAGE;
        }
        @Override
        public boolean isFinished(){
                return false;
        }
}
