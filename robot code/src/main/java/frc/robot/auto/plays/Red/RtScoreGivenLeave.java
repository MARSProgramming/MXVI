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
            new ResetDrivePose(drivetrain, 14.71, 4.36, 0),
            new ParallelCommandGroup(
                // Move the arm to scoring position
                // Add stuff here to score given piece
                // Possibly add stuff here to move bot to an ideal position                
            )
         );
    }
}
