package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class StartToScoreRight extends SequentialCommandGroup{
    public StartToScoreRight(Arm arm){
        addCommands(
            new GoToAngles(arm, Math.PI/2, -Math.PI/2),
            new MoveArmToPoint(arm, 0.8, 0.5)
        );
    }
}
