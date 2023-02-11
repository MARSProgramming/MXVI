package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RetractClaw extends CommandBase{
    private Arm mArm;

    public RetractClaw(Arm sub){
        mArm = sub;
    }

    @Override
    public void initialize(){
        mArm.retract();
    }

    @Override
    public void end(boolean interrupted){
        
    }
}
