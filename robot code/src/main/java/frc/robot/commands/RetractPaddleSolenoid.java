package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Paddle;

public class RetractPaddleSolenoid extends CommandBase{
    private Paddle mPaddle;

    public RetractPaddleSolenoid(Paddle sub){
        mPaddle = sub;
    }

    @Override
    public void initialize(){
        mPaddle.retract();
    }

    @Override
    public void end(boolean interrupted){
        
    }
}
