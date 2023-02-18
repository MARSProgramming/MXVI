package frc.robot.commands.Arm.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class StartToLoad extends SequentialCommandGroup{
    public StartToLoad(Arm arm){
        addCommands(new GoToShoulderFirst(arm, 0.0, 3.4), new GoToShoulderFirst(arm, -0.5, 3.4));
    }
}
