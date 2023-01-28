package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroGyroscope extends CommandBase{
    private DrivetrainSubsystem mDT;
    public ZeroGyroscope(DrivetrainSubsystem dt){
        mDT = dt;
        addRequirements(dt);
    }

    @Override
    public void initialize(){
        mDT.zeroGyroscope();
    }
}
