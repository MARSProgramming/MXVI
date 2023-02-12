package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends CommandBase{
    private IntakeSubsystem mIntake;
    public Intake(IntakeSubsystem sub){
        mIntake = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize(){
        mIntake.runIntakePOutput(0.0);
    }

    @Override
    public void execute(){
        mIntake.runIntakePOutput(0.9);
    }

    @Override
    public void end(boolean interrupted){
        mIntake.runIntakePOutput(0.0);
    }
}
