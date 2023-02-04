package frc.robot.auto.plays.Red;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RSetMidPose extends SequentialCommandGroup{
    public RSetMidPose(DrivetrainSubsystem drivetrain){
        addRequirements(drivetrain);

        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 2.69, 0),
            new ParallelCommandGroup(

            )
        );
    }
}
