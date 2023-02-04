package frc.robot.auto.plays.Red;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RtP1P2_NoDock extends SequentialCommandGroup{
    public RtP1P2_NoDock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        Trajectory MarkertoP1 = AutoChooser.openTrajectoryFile("RED_TopMarker_M-P1.wpilib.json");
        Trajectory P1toMarker = AutoChooser.openTrajectoryFile("RED_TopMarker_P1-M.wpilib.json");
        Trajectory MarkertoP2 = AutoChooser.openTrajectoryFile("RED_TopMarker_M-P2.wpilib.json");
        Trajectory P2toMarker = AutoChooser.openTrajectoryFile("RED_TopMarker_P2-M.wpilib.json");
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 4.36, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP1, new Rotation2d(0), 10),
                new DriveAtPath(drivetrain, P1toMarker, new Rotation2d(0), 10),
                new DriveAtPath(drivetrain, MarkertoP2, new Rotation2d(0), 10),
                new DriveAtPath(drivetrain, P2toMarker, new Rotation2d(0), 10)

            )

        );
    }
}
