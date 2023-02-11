package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomSolenoids extends SubsystemBase {
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(61, PneumaticsModuleType.REVPH, 1, 5);

    public BottomSolenoids() {
    }
    public void extend(){
        mSolenoid.set(Value.kForward);
      }
      public void retract(){  
        mSolenoid.set(Value.kReverse);
      }
      public void toggle(){
        if(mSolenoid.get() == Value.kOff){mSolenoid.set(Value.kForward);}
        mSolenoid.toggle();
      }

    @Override
    public void periodic() {
      SmartDashboard.putString("BottomSolenoidState", mSolenoid.get().toString());
    }
}
