package frc.robot.auto.plays.Red;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RbP4_Dock extends SequentialCommandGroup{
    public RbP4_Dock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        Trajectory MarkertoP4 = AutoChooser.openTrajectoryFile("RED_BottomMarker_M-P4.wpilib.json");
        Trajectory P4toMarker = AutoChooser.openTrajectoryFile("RED_BottomMarker_P4-M.wpilib.json");
        Trajectory MarkerToDock = AutoChooser.openTrajectoryFile("RED_BottomMarker_M-C.wpilib.json");
        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 0.45, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP4, 0, 10),
                new DriveAtPath(drivetrain, P4toMarker, 0, 10),
                new DriveAtPath(drivetrain, MarkerToDock, 0, 10)

            )

        );
    }
}
