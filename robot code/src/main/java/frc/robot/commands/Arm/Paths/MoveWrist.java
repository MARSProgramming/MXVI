package frc.robot.commands.Arm.Paths;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveWrist extends CommandBase{
    private Arm mArm;
    private double pos;
    public MoveWrist(Arm sub, double pos){
        this.pos = pos;
        mArm = sub;
    }

    @Override
    public void execute(){
        mArm.setWristPosition(pos);
    }

    @Override
    public void end(boolean interrupted){
        mArm.runWrist(0);
    }
}
