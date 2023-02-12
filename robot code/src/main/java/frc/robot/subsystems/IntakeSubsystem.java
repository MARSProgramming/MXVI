package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
      public void toggle(){
        if(mSolenoid.get() == Value.kOff){mSolenoid.set(Value.kReverse);}
         mSolenoid.toggle();
      }
    public void RunMotors(double voltage) {
      if (Motor0.getStatorCurrent() > 1) {
        Motor0.set(ControlMode.PercentOutput, 0);
      } else{
        Motor0.set(ControlMode.PercentOutput, voltage);
      }

    }
    @Override
    public void periodic() {
      SmartDashboard.putString("IntakeSolenoid State", mSolenoid.get().toString());
    }
    public void runIntakePOutput(double d) {
      Motor0.set(ControlMode.PercentOutput, d);
    }
    public Command toggleIntake() {
        return null;
    }
    public Command runIntakeMotors(Object object) {
        return null;
    }
}