package frc.robot.commands.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class GoToAngles extends CommandBase{
    private Arm mArm;
    private double angle1;
    private double angle2;
    public GoToAngles(Arm sub, double a1, double a2){
        mArm = sub;
        angle1 = a1;
        angle2 = a2;
    }

    @Override
    public void execute(){
        mArm.goToAngles(angle1, angle2);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(mArm.getShoulderPosition() - Units.radiansToDegrees(angle1)) < 3 && Math.abs(mArm.getElbowPosition() - Units.radiansToDegrees(angle2)) < 3;
    }
}
