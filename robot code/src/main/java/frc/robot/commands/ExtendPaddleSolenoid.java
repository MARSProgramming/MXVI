package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Paddle;

public class ExtendPaddleSolenoid extends CommandBase{
    private Paddle mPaddle;

    public ExtendPaddleSolenoid(Paddle sub){
        mPaddle = sub;
    }

    @Override
    public void initialize(){
        mPaddle.extend();
    }

    @Override
    public void end(boolean interrupted){
        
    }
}
