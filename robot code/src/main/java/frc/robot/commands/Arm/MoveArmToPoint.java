package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArmToPoint extends CommandBase{
    private Arm mArm;
    private DoubleSupplier mX;
    private DoubleSupplier mY;
    public MoveArmToPoint(Arm a, DoubleSupplier x, DoubleSupplier y){
        mX = x;
        mY = y;
        mArm = a;
        addRequirements(a);
    }

    @Override
    public void execute(){
        mArm.runAtVelocity(-mX.getAsDouble(), mY.getAsDouble());
    }
    @Override
    public void end(boolean interrupted){
        mArm.runElbowPOutput(0);
        mArm.runShoulderPOutput(0);
    }
}
