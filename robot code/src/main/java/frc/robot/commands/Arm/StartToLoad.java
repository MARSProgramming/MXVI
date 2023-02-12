package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class StartToLoad extends SequentialCommandGroup{
    public StartToLoad(Arm arm){
        addCommands(new GoToElbowFirst(arm, -0.4, 3.88));
    }
}
