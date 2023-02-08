package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
public class IntakeToggleCommand extends CommandBase {
    private IntakeSubsystem mIntakeSub;
    public IntakeToggleCommand(IntakeSubsystem i) {
        mIntakeSub = i;
        addRequirements(i);
    }
    @Override
  public void initialize() {
    System.out.println("Extended");
    mIntakeSub.toggle();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}














