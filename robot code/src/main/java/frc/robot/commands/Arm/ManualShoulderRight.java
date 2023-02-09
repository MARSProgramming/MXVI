package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualShoulderRight extends CommandBase{
    private Arm mArm;
    public ManualShoulderRight(Arm a){
        mArm = a;
        addRequirements(a);
    }

    @Override
    public void execute(){
        mArm.runShoulderPOutput(0.5);
    }

    @Override
    public void end(boolean interrupted){
        mArm.runShoulderPOutput(0);
    }
}
