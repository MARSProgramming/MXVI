package frc.robot.auto.plays.Blue;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BBottomLeaveCommunity extends SequentialCommandGroup{
    public BBottomLeaveCommunity(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        Trajectory LeaveCommunity = AutoChooser.openTrajectoryFile("BLUE_BottomLeaveCommunity.wpilib.json");
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 0.45, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, LeaveCommunity, 0, 10)
            )

        );
    }
}
