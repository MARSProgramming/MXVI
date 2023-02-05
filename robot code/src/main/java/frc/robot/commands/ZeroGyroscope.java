package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroGyroscope extends CommandBase{
    private DrivetrainSubsystem mDT;
    private double zero;
    public ZeroGyroscope(DrivetrainSubsystem dt, double zero){
        mDT = dt;
        this.zero = zero;
        addRequirements(dt);
    }

    @Override
    public void initialize(){
        mDT.zeroGyroscope(zero);
    }
}
