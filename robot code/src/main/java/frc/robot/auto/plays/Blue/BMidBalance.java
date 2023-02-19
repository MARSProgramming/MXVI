package frc.robot.auto.plays.Blue;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.ResetDrivePose;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

public class BMidBalance extends SequentialCommandGroup{
    public BMidBalance(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        Trajectory LeaveCommunity = AutoChooser.openTrajectoryFile("BLUE_MiddleMarker_M-C.wpilib.json");
        addCommands(
            new ZeroGyroscope(drivetrain, 0).withTimeout(0.1),
            new ResetDrivePose(drivetrain, 1.81, 2.69, 0),
            new ParallelCommandGroup(
                // Any changes to make sure arm is secure here
                new DriveAtPath(drivetrain, LeaveCommunity, 0, 10)
            )

        );
    }
}
