package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualShoulderLeft extends CommandBase{
    private Arm mArm;
    public ManualShoulderLeft(Arm a){
        mArm = a;
        addRequirements(a);
    }

    @Override
    public void execute(){
        mArm.runShoulderPOutput(-0.1);
    }

    @Override
    public void end(boolean interrupted){
        mArm.runShoulderPOutput(0);
    }
}
