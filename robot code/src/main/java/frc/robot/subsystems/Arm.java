package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(61, PneumaticsModuleType.REVPH, 0, 4);
    private TalonFX mElbow = new TalonFX(Constants.Arm.kElbowMotorID);
    private TalonFX mShoulder = new TalonFX(Constants.Arm.kShoulderMotorID);
    private TalonSRX mWrist = new TalonSRX(13);

    private double talonFXSensorCoefficient = 2048;
    private final double angularVelocityCoefficient = 2048 * 10 / 2 / Math.PI;
    public Arm(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        mElbow.configAllSettings(config);
        mShoulder.configAllSettings(config);

        mElbow.setSelectedSensorPosition(0, 0, 0);
        mShoulder.setSelectedSensorPosition(0, 0, 0);

        mElbow.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        mShoulder.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        mElbow.config_kP(0, 0.00001);
        mShoulder.config_kP(0, 0.00001);
        mElbow.config_kI(0, 0.0);
        mShoulder.config_kI(0, 0.0);
        mElbow.config_kD(0, 0.0);
        mShoulder.config_kD(0, 0.0);

        mElbow.setNeutralMode(NeutralMode.Brake);
        mShoulder.setNeutralMode(NeutralMode.Brake);
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
    public CommandBase toggleClaw() {
        return runOnce(
            () -> {
            if (mSolenoid.get()==Value.kOff)
                mSolenoid.set(Value.kReverse); 
            else
                mSolenoid.toggle();
        }).withName("Test Claw Pneumatics");
    }

    @Override
    public void periodic() {
      SmartDashboard.putString("ClawSolenoidState", mSolenoid.get().toString());
    }
    public void setShoulderVelocity(double deg){
        mShoulder.set(ControlMode.Velocity, deg * talonFXSensorCoefficient);
    }

    public void setElbowVelocity(double deg){
        mElbow.set(ControlMode.Velocity, deg * angularVelocityCoefficient);
    }

    public void runElbowPOutput(double v){
        mElbow.set(ControlMode.PercentOutput, v);
    }

    public void runShoulderPOutput(double v){
        mShoulder.set(ControlMode.PercentOutput, v);
    }

    public void runWrist(double v){
        mWrist.set(ControlMode.PercentOutput, v);
    }

    public void initializeSolenoid(){
        mSolenoid.set(Value.kReverse);
    }
}
