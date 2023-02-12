package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualElbowRight extends CommandBase{
    private Arm mArm;
    public ManualElbowRight(Arm a){
        mArm = a;
        addRequirements(a);
    }

    @Override
    public void execute(){
        mArm.runElbowPOutput(0.1);
    }

    @Override
    public void end(boolean interrupted){
        mArm.runElbowPOutput(0);
    }
}
