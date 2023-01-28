package frc.robot.auto.plays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class TestAutoPlay extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;
    public TestAutoPlay(DrivetrainSubsystem drivetrain){
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);

        Trajectory lPath1 = AutoChooser.openTrajectoryFile("test.wpilib.json");
        Trajectory lPath2 = AutoChooser.openTrajectoryFile("testForwardPath.wpilib.json");
        addCommands(
            new ResetDrivePose(mDrivetrain, 1.7, 0.8, 0),
            new ParallelCommandGroup(
                new DriveAtPath(mDrivetrain, lPath1, new Rotation2d(3.14159), 10)
            )
            /*new ShooterCommand(mShooter, 2.0),
            new ParallelCommandGroup(
                new DriveAtPath(mDrivetrain, lPath2, new Rotation2d(0.0), 2.0), new ShooterCommand(mShooter, 2.0)
            ),
            new ShooterCommand(mShooter, 2.0)*/
            //new DriveAtPath(mDrivetrain, lPath1, new Rotation2d(Math.toRadians(185.0)))
            //new DriveAtPath(mDrivetrain, lPath2, new Rotation2d(0.0))
        );
    }
}
