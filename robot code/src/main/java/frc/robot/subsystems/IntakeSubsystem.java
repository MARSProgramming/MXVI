package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class IntakeSubsystem extends SubsystemBase {
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(61, PneumaticsModuleType.REVPH, 2, 6);
    private TalonFX Motor0 = new TalonFX(12, "Drivetrain");
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
      public void toggle(){
        if(mSolenoid.get() == Value.kOff){mSolenoid.set(Value.kReverse);}
         mSolenoid.toggle();
      }
    public void RunMotors(double voltage) {
        Motor0.set(ControlMode.PercentOutput, voltage);
    }
    @Override
    public void periodic() {
      SmartDashboard.putString("IntakeSolenoid State", mSolenoid.get().toString());
    }
}