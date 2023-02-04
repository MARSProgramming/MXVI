package frc.robot.auto.plays.Red;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RtScoreGiven extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;
    public RtScoreGiven(DrivetrainSubsystem drivetrain){
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);

        addCommands(
            new ResetDrivePose(drivetrain, 14.71, 4.37, 0),
            new ParallelCommandGroup(
                // Add stuff here to score given piece
            )
         );
    }
}
