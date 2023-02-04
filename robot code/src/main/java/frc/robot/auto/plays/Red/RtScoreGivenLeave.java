package frc.robot.auto.plays.Red;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RtScoreGivenLeave extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;
    public RtScoreGivenLeave(DrivetrainSubsystem drivetrain){
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);
        // Add stuff here to leave community
        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 4.36, 0),
            new ParallelCommandGroup(
                // Add stuff here to score given piece
                // Add more 
            )
         );
    }
}
