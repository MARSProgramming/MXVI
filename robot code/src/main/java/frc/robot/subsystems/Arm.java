package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private DoubleSolenoid mSolenoid = new DoubleSolenoid(61, PneumaticsModuleType.REVPH, 0, 4);
    /*private TalonFX mElbow;
    private TalonFX mShoulder;

    private double talonFXSensorCoefficient = 2048;*/
    public Arm(){
        /*mElbow = new TalonFX(Constants.Arm.kElbowMotorID);
        mShoulder = new TalonFX(Constants.Arm.kShoulderMotorID);

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
        mElbow.config_kF(0, 0.0);
        mShoulder.config_kF(0, 0.0);

        mElbow.setNeutralMode(NeutralMode.Brake);
        mShoulder.setNeutralMode(NeutralMode.Brake);*/
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

    @Override
    public void periodic() {
      SmartDashboard.putString("ClawSolenoidState", mSolenoid.get().toString());
    }
    /*public void setShoulderAngle(double deg){
        mShoulder.set(ControlMode.Position, deg * talonFXSensorCoefficient);
    }

    public void setElbowAngle(double deg){
        mElbow.set(ControlMode.Position, deg * talonFXSensorCoefficient);
    }

    public void runElbowPOutput(double v){
        mElbow.set(ControlMode.PercentOutput, v);
    }

    public void runShoulderPOutput(double v){
        mShoulder.set(ControlMode.PercentOutput, v);
    }*/
}
