package frc.robot.commands.Arm.Paths;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveVelocity extends CommandBase{
    Arm mArm;
    DoubleSupplier x;
    DoubleSupplier y;
    public MoveVelocity(Arm sub, DoubleSupplier x, DoubleSupplier y){
        this.x = x;
        this.y = y;
        mArm = sub;
        addRequirements(sub);
    }
    
    @Override
    public void execute(){
        mArm.runAtVelocity(x.getAsDouble(), -y.getAsDouble());
        System.out.println("running");
        //SmartDashboard.putNumber("runnning", 011);
    }

    @Override
    public void end(boolean interrupted){
        mArm.runAtVelocity(0, 0);
    }
}
