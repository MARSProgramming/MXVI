package frc.robot.auto.plays.Blue;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BbP4_NoDock extends SequentialCommandGroup{
    public BbP4_NoDock(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        Trajectory MarkertoP4 = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_M-P4.wpilib.json");
        Trajectory P4toMarker = AutoChooser.openTrajectoryFile("BLUE_BottomMarker_P4-M.wpilib.json");
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 0.45, 0),
            new ParallelCommandGroup(
                new DriveAtPath(drivetrain, MarkertoP4,0, 10),
                new DriveAtPath(drivetrain, P4toMarker, 0, 10)

            )

        );
    }
}
