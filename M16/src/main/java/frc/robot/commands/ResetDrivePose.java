package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ResetDrivePose extends CommandBase{
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private double mX;
    private double mY;
    private double mRotation;
    public ResetDrivePose(DrivetrainSubsystem driveSubsystem, double x, double y, double rotation){
        mDrivetrainSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        mX = x;
        mY = y;
        mRotation = rotation;
    }
    @Override
    public void initialize(){
        Rotation2d rotation = new Rotation2d(mRotation);
        mDrivetrainSubsystem.setPose(new Pose2d(mX, mY, rotation), rotation);
    }
    @Override
    public boolean isFinished(){
        return mDrivetrainSubsystem.getPose().getTranslation().getDistance(new Translation2d(mX, mY)) < 0.01;
    }
}
