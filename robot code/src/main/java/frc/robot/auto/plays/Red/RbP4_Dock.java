package frc.robot.auto.plays.Blue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BbP4_Dock extends SequentialCommandGroup{
    public BbP4_Dock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        Trajectory MarkertoP4 = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-P4.wpilib.json");
        Trajectory P4toMarker = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_P4-M.wpilib.json");
        Trajectory MarkerToDock = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-C.wpilib.json");
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 0.45, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP4, new Rotation2d(0), 10),
                new DriveAtPath(drivetrain, P4toMarker, new Rotation2d(0), 10),
                new DriveAtPath(drivetrain, MarkerToDock, new Rotation2d(0), 10)

            )

        );
    }
}