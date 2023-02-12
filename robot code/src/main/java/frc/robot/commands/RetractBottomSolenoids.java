package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomSolenoids;

public class RetractBottomSolenoids extends CommandBase{
    private BottomSolenoids mBottomSolenoids;

    public RetractBottomSolenoids(BottomSolenoids sub){
        mBottomSolenoids = sub;
    }

    @Override
    public void initialize(){
        mBottomSolenoids.retract();
    }

    @Override
    public void end(boolean interrupted){

    }
}

