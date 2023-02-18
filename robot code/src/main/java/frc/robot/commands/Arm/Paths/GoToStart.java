package frc.robot.commands.Arm.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class GoToStart extends SequentialCommandGroup{
    public GoToStart(Arm arm){
        addCommands(new GoToAngles(arm, Math.PI/2, Math.PI/2), new GoToElbowFirst(arm, -Math.PI/2, Math.PI));
    }
}
