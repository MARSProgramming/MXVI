package frc.robot.auto.plays.Red;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class RtP2_Dock extends SequentialCommandGroup{
    public RtP2_Dock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        Trajectory MarkertoP2 = AutoChooser.openTrajectoryFile("RED_TopMarker_M-P2.wpilib.json");
        Trajectory P2toMarker = AutoChooser.openTrajectoryFile("RED_TopMarker_P2-M.wpilib.json");
        Trajectory MarkerToDock = AutoChooser.openTrajectoryFile("RED_TopMarker_M-C.wpilib.json");
        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 4.37, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP2, 0, 10),
                new DriveAtPath(drivetrain, P2toMarker, 0, 10),
                new DriveAtPath(drivetrain, MarkerToDock, 0, 10)

            )

        );
    }
}
