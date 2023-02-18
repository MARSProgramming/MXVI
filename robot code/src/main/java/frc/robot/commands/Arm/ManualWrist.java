package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualWrist extends CommandBase{
    private Arm mArm;
    public ManualWrist(Arm a){
        mArm = a;
        addRequirements(a);
    }

    @Override
    public void execute(){
        mArm.runWrist(-0.15);
    }

    @Override
    public void end(boolean interrupted){
        mArm.runWrist(0);
    }
}
