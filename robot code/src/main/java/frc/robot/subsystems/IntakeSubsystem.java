package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class IntakeSubsystem extends SubsystemBase {
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(61, PneumaticsModuleType.REVPH, 2, 6);
    private TalonFX Motor0 = new TalonFX(12);
    public IntakeSubsystem() {
        Motor0.setNeutralMode(NeutralMode.Coast);
        Motor0.setInverted(false);
        mSolenoid.set(Value.kReverse);
    }
    public void extend(){
      mSolenoid.set(Value.kForward);
    }
    public void retract(){
      mSolenoid.set(Value.kReverse);
    }

    public CommandBase toggleIntake() {
      return runOnce(
        () -> {
          if (mSolenoid.get()==Value.kOff)
            mSolenoid.set(Value.kReverse); 
          else
            mSolenoid.toggle();
        }).withName("Test Intake Pneumatics");
    }
    public CommandBase runIntakeMotors(DoubleSupplier percent) {
      return run(() -> Motor0.set(ControlMode.PercentOutput, percent.getAsDouble()))
        .withName("Test Intake Motor");
    }

    public void runIntakePOutput(double v){
      Motor0.set(ControlMode.PercentOutput, v);
    }
    
    @Override
    public void periodic() {
      SmartDashboard.putString("IntakeSolenoid State", mSolenoid.get().toString());
    }
}