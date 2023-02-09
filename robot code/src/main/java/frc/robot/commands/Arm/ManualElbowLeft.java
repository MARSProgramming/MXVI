package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualElbowLeft extends CommandBase{
    private Arm mArm;
    public ManualElbowLeft(Arm a){
        mArm = a;
        addRequirements(a);
    }

    @Override
    public void execute(){
        mArm.runElbowPOutput(-0.5);
    }

    @Override
    public void end(boolean interrupted){
        mArm.runElbowPOutput(0);
    }
}
