package frc.robot.auto.plays.Blue;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BbP3P4_Dock extends SequentialCommandGroup{
    public BbP3P4_Dock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);
        Trajectory MarkertoP3 = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-P3.wpilib.json");
        Trajectory P3toMarker = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_P3-M.wpilib.json");
        Trajectory MarkertoP4 = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-P4.wpilib.json");
        Trajectory P4toMarker = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_P4-M.wpilib.json");
        Trajectory MarkerToDock = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-C.wpilib.json");
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 0.45, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP3, 0, 10),
                new DriveAtPath(drivetrain, P3toMarker, 0, 10),
                new DriveAtPath(drivetrain, MarkertoP4, 0, 10),
                new DriveAtPath(drivetrain, P4toMarker, 0, 10),
                new DriveAtPath(drivetrain, MarkerToDock, 0, 10)

            )

        );
    }
}
