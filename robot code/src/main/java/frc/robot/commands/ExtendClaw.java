package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtendClaw extends CommandBase{
    private Arm mArm;

    public ExtendClaw(Arm sub){
        mArm = sub;
    }

    @Override
    public void initialize(){
        mArm.extend();
    }

    @Override
    public void end(boolean interrupted){
        
    }
}
