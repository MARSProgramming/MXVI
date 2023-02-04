package frc.robot.auto.plays.Blue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BtP1_NoDock extends SequentialCommandGroup{
    public BtP1_NoDock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        Trajectory MarkertoP1 = AutoChooser.openTrajectoryFile("BLUE_TopMarker_M-P1.wpilib.json");
        Trajectory P1toMarker = AutoChooser.openTrajectoryFile("BLUE_TopMarker_P1-M.wpilib.json");
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 4.36, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP1, new Rotation2d(0), 10),
                new DriveAtPath(drivetrain, P1toMarker, new Rotation2d(0), 10)

            )

        );
    }
}
