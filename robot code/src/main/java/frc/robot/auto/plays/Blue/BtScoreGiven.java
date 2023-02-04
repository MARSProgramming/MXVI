package frc.robot.auto.plays.Blue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetDrivePose;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BtScoreGiven extends SequentialCommandGroup{
    private DrivetrainSubsystem mDrivetrain;
    public BtScoreGiven(DrivetrainSubsystem drivetrain){
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);

        addCommands(
            new ResetDrivePose(drivetrain, 1.81, 4.36, 0),
            new ParallelCommandGroup(
                // Add stuff here to score given piece
            )
         );
    }
}
