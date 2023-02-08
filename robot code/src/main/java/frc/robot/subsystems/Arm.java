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

        mElbow.setNeutralMode(NeutralMode.Brake);
        mShoulder.setNeutralMode(NeutralMode.Brake);
        mWrist.setNeutralMode(NeutralMode.Brake);
        mWrist.config_kP(0, kP);
        mWrist.config_kI(0, kI);
        mWrist.config_kD(0, kI);
        mWrist.set(ControlMode.Position, 0);
       
    }

    public void runElbowPOutput(double v){
        mElbow.set(ControlMode.PercentOutput, v);
    }

    public void setElbowPosition(double v){
        mElbow.set(ControlMode.Position, v * 177);
    }


    public void runShoulderPOutput(double v){
        mShoulder.set(ControlMode.PercentOutput, v);
    }

    public void setShoulderPosition(double v){
        mShoulder.set(ControlMode.Position, v * 177);
    }

    public void runWristPOutput(double v){
        mWrist.set(ControlMode.PercentOutput, v);
    }

    public void setWristPosition(double v){
        mWrist.set(ControlMode.Position, v * 177);
    }

    @Override
    public void periodic(){
       wristPosition = mWrist.getSelectedSensorPosition()* 2*Math.PI/4096;
    }
    }
