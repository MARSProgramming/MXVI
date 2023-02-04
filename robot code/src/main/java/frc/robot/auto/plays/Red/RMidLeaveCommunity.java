package frc.robot.auto.plays.Red;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RMidLeaveCommunity extends SequentialCommandGroup{
    public RMidLeaveCommunity(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        Trajectory LeaveCommunity = AutoChooser.openTrajectoryFile("RED_MiddleLeaveCommunity.wpilib.json");
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 2.69, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, LeaveCommunity, new Rotation2d(0), 10)
            )

        );
    }
}
