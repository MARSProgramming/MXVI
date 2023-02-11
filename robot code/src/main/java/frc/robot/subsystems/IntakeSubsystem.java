package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class IntakeSubsystem extends SubsystemBase {
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(63, PneumaticsModuleType.REVPH, 8, 9);
    private TalonFX Motor0 = new TalonFX(12);
    public IntakeSubsystem() {
        Motor0.setNeutralMode(NeutralMode.Coast);
        Motor0.setInverted(false);
        mSolenoid.set(Value.kReverse);
    }
    /**
     * Returns a command that toggles double solenoid.
     */
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
      return run(() -> Motor0.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Intake Motor Speed", percent.getAsDouble())))
        .withName("Test Intake Motor");
    }
    @Override
    public void periodic() {
      SmartDashboard.putString("Solenoid State", mSolenoid.get().toString());
    }
}