package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Arm extends SubsystemBase implements Loggable{
    private TalonFX mElbow;
    private TalonFX mShoulder;
    private TalonFX mWrist;
    @Log
    private double wristPosition;
        
    public Arm(){

        mElbow = new TalonFX(Constants.Arm.kElbowMotorID);
        mShoulder = new TalonFX(Constants.Arm.kShoulderMotorID);
        mWrist = new TalonFX(Constants.Arm.kWristMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        mElbow.configAllSettings(config);
        mShoulder.configAllSettings(config);
        mWrist.configAllSettings(config);

        mElbow.setNeutralMode(NeutralMode.Brake);
        mShoulder.setNeutralMode(NeutralMode.Brake);
        mWrist.setNeutralMode(NeutralMode.Brake);
    }

    public void runElbowPOutput(double v){
        mElbow.set(ControlMode.PercentOutput, v);
    }

    public void runShoulderPOutput(double v){
        mShoulder.set(ControlMode.PercentOutput, v);
    }

    public void runWristPOutput(double v){
        mWrist.set(ControlMode.PercentOutput, v);
    }

    @Override
    public void periodic(){
       wristPosition = mWrist.getSelectedSensorPosition()* 2*Math.PI/4096;
    }
    }
