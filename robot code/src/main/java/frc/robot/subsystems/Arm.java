package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    private TalonFX mElbow;
    private TalonFX mShoulder;
    public Arm(){
        mElbow = new TalonFX(Constants.Arm.kElbowMotorID);
        mShoulder = new TalonFX(Constants.Arm.kShoulderMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        mElbow.configAllSettings(config);
        mShoulder.configAllSettings(config);

        mElbow.setNeutralMode(NeutralMode.Brake);
        mShoulder.setNeutralMode(NeutralMode.Brake);
    }

    public void runElbowPOutput(double v){
        mElbow.set(ControlMode.PercentOutput, v);
    }

    public void runShoulderPOutput(double v){
        mShoulder.set(ControlMode.PercentOutput, v);
    }
}
