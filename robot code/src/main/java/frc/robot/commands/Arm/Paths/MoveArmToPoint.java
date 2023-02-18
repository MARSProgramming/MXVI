package frc.robot.commands.Arm.Paths;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArmToPoint extends CommandBase{
    private Arm mArm;
    private double x;
    private double y;
    public MoveArmToPoint(Arm a, double x, double y){
        this.x = x;
        this.y = y;
        mArm = a;
        addRequirements(a);
        xController.reset(mArm.getPosition()[0]);
        yController.reset(mArm.getPosition()[1]);
    }

    ProfiledPIDController xController = new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(0.5, 1));
    ProfiledPIDController yController = new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(0.5, 1));

    @Override
    public void execute(){
        //double nextEndX = mArm.getPosition()[0] + mX.getAsDouble()*0.1;
        //double nextEndY = mArm.getPosition()[1] + mY.getAsDouble()*0.1;
        //mArm.goToAngles(Math.PI/2, -Math.PI/2);
        //mArm.runAtVelocity(mX.getAsDouble()/5, -mY.getAsDouble()/5);
        //mArm.runAtVelocity(MathUtil.clamp(xController.calculate(mArm.getPosition()[0], x), -0.05, 0.05), MathUtil.clamp(yController.calculate(mArm.getPosition()[1], y), -0.05, 0.05));
        SmartDashboard.putNumber("x", mArm.getPosition()[0]);
        SmartDashboard.putNumber("y", mArm.getPosition()[1]);
        //System.out.println(MathUtil.clamp(xController.calculate(mArm.getPosition()[0], x), -0.1, 0.1));
    }
    @Override
    public void end(boolean interrupted){
        mArm.runElbowPOutput(0);
        mArm.runShoulderPOutput(0);
    }
}
