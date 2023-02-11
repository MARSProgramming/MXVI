package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Paddle extends SubsystemBase{
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(61, PneumaticsModuleType.REVPH,3, 7);
    
    public Paddle(){
        
    }

    public void extend(){
        mSolenoid.set(Value.kForward);
    }
    public void retract(){
        mSolenoid.set(Value.kReverse);
    }
    public void toggle(){
        mSolenoid.toggle();
    }

    @Override
    public void periodic() {
      SmartDashboard.putString("PaddleSolenoidState", mSolenoid.get().toString());
    }
}
