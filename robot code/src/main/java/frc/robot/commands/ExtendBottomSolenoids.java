package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomSolenoids;

public class ExtendBottomSolenoids extends CommandBase{
    private BottomSolenoids mBottomSolenoids;

    public ExtendBottomSolenoids(BottomSolenoids sub){
        mBottomSolenoids = sub;
    }

    @Override
    public void initialize(){
        mBottomSolenoids.extend();
    }

    @Override
    public void end(boolean interrupted){
        
    }
}
