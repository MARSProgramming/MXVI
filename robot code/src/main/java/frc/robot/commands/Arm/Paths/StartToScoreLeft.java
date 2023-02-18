package frc.robot.commands.Arm.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class StartToScoreLeft extends SequentialCommandGroup{
    public StartToScoreLeft(Arm arm){
        addCommands(
            new GoToShoulderFirst(arm, Math.PI/2, Math.PI),
            new GoToAngles(arm, Math.PI/2 + 0.3, Math.PI + 0.4)
            //new MoveArmToPoint(arm, -0.8, 0.5)
        );
    }
}
