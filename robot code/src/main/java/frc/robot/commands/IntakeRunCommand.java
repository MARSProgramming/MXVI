package frc.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
public class IntakeRunCommand extends CommandBase {
    private IntakeSubsystem mIntakeSub;
    private DoubleSupplier output;
    public IntakeRunCommand(IntakeSubsystem i, DoubleSupplier ds) {
        mIntakeSub = i;
        output = ds;
        addRequirements(i);
    }
    @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntakeSub.RunMotors(output.getAsDouble());
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntakeSub.RunMotors(0);
  }
}
