package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.util.Units;
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

    private static final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
    private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / kCountsPerRev;
    final int k100msPerSecond = 10;
    private final double angularVelocityCoefficient = 2048 * 10 / 2 / Math.PI * 54;

    private static final double kUpperArmLength = Units.inchesToMeters(23);

    private static final double kLowerArmLength = Units.inchesToMeters(23);

    private static final double kWristLength = Units.inchesToMeters(8);

    private static final double kShoulderGearRatio = 53;
    private static final double kElbowGearRatio = 53;
    public Arm(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        mElbow.configAllSettings(config);
        mShoulder.configAllSettings(config);

        mElbow.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        mShoulder.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        mElbow.setSelectedSensorPosition(rotationToNativeUnits(-Math.PI/2), 0, 0);
        mShoulder.setSelectedSensorPosition(rotationToNativeUnits(-Math.PI/2), 0, 0);
        mElbow.setInverted(true);

        mElbow.config_kP(0, 0.03);
        mShoulder.config_kP(0, 0.03);
        mElbow.config_kI(0, 0.0);
        mShoulder.config_kI(0, 0.0);
        mElbow.config_kD(0, 0.0);
        mShoulder.config_kD(0, 0.0);
        mShoulder.configPeakOutputForward(0.2);
        mElbow.configPeakOutputForward(0.2);

        mShoulder.configClosedloopRamp(3);
        mElbow.setNeutralMode(NeutralMode.Brake);
        mShoulder.setNeutralMode(NeutralMode.Brake);
        mWrist.configFactoryDefault();
        mWrist.setNeutralMode(NeutralMode.Brake);
        mWrist.setSelectedSensorPosition(0, 1, 0);
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

    public void goToAngles(double a1, double a2){
        goToShoulder(a1);
        goToElbow(a2);
    }

    public void goToShoulder(double a){
        double radians = java.lang.Math.toRadians(getShoulderPosition());
        double cosineScalar = java.lang.Math.cos(radians);
        mShoulder.set(ControlMode.Position, rotationToNativeUnits(a), DemandType.ArbitraryFeedForward,cosineScalar * 0.2);
    }

    public void goToElbow(double a){
        double radians = java.lang.Math.toRadians(getElbowPosition());
        double cosineScalar = java.lang.Math.cos(radians);
        mElbow.set(ControlMode.Position, rotationToNativeUnits(a), DemandType.ArbitraryFeedForward,cosineScalar * 0.2);
    }

    @Override
    public void periodic() {
      SmartDashboard.putString("ClawSolenoidState", mSolenoid.get().toString());
      SmartDashboard.putNumber("Shoulder pos(deg)", getShoulderPosition());
      SmartDashboard.putNumber("Elbow pos(deg)", getElbowPosition());
      SmartDashboard.putNumber("Wrist Pos", mWrist.getSelectedSensorPosition()/177.6 * 2 * Math.PI);
      //SmartDashboard.putNumber("X", getPosition()[0]);
      //SmartDashboard.putNumber("Y", getPosition()[1]);
    }

    public double getShoulderPosition(){
        return Units.radiansToDegrees(nativeUnitsToRotationRad(mShoulder.getSelectedSensorPosition()));
    }

    public double getElbowPosition(){
        return Units.radiansToDegrees(nativeUnitsToRotationRad(mElbow.getSelectedSensorPosition()));
    }

    public double[] getPosition(){
        double l1 = kLowerArmLength;
        double l2 = kUpperArmLength;
        double[] pos = new double[2];
        double th = Units.degreesToRadians(getShoulderPosition());
        double phi = Units.degreesToRadians(getElbowPosition());
  
        pos[0] = l1 * Math.cos(th) + l2 * Math.cos(th + phi);
        pos[1] = l1 * Math.sin(th) + l2 * Math.sin(th + phi);
  
        return pos;
    }

    public double[] getElbowJointPos(){
        double x = kLowerArmLength * Math.cos(Units.degreesToRadians(getShoulderPosition()));
        double y = kLowerArmLength * Math.sin(Units.degreesToRadians(getShoulderPosition()));
  
        double[] pos = new double[2];
        pos[0] = x;
        pos[1] = y;
        return pos;
    }
  
    public void runAtVelocity(double xdot, double ydot){
        double l1 = kLowerArmLength;
        double l2 = kUpperArmLength;
  
        double th = Units.degreesToRadians(getShoulderPosition()) + 0.000001;
        double phi = Units.degreesToRadians(getElbowPosition()) + 0.0000001;
  
        System.out.println("adj ang " + th + " " + (th + phi));
  
        double a = -l1 * Math.sin(th) - l2 * Math.sin(th + phi);
        double b = -l2 * Math.sin(th + phi);
        double c = l1 * Math.cos(th) + l2 * Math.cos(th + phi);
        double d = l2 * Math.cos(th + phi);
  
        System.out.println(th + " " + phi);
  
        double[][] Adouble = {{a, b}, {c, d}};
        SimpleMatrix A = new SimpleMatrix(Adouble);
        
        SimpleMatrix Xd = new SimpleMatrix(2, 1);
        Xd.set(0, 0, Math.copySign(Math.min(Math.abs(xdot), 0.01), -xdot));
        Xd.set(1, 0, Math.copySign(Math.min(Math.abs(ydot), 0.01), -ydot));
  
        System.out.println(A.invert());
        SimpleMatrix B = A.invert().mult(Xd);
        System.out.println(B);
        //if(getPosition()[1] > -0.5){
          //setShoulderVelocity(-B.get(0, 0) * 2048 * k100msPerSecond / 2 / Math.PI * kShoulderGearRatio);
          //setElbowVelocity(-B.get(1, 0) * 2048 * k100msPerSecond / 2 / Math.PI * kElbowGearRatio);
        //}
        /*else{
          mShoulder.set(ControlMode.PercentOutput, 0);
          mElbow.set(ControlMode.PercentOutput, 0);
        }*/
        SmartDashboard.putNumber("B", Units.radiansToDegrees(-B.get(0, 0)));
        SmartDashboard.putNumber("A", Units.radiansToDegrees(-B.get(1, 0)));
        //System.out.println("B " + -B.get(0, 0) + " " + -B.get(1, 0));
    }

    public void setShoulderVelocity(double deg){
        double radians = java.lang.Math.toRadians(getShoulderPosition());
        double cosineScalar = java.lang.Math.cos(radians);
        mShoulder.set(ControlMode.Velocity, deg * angularVelocityCoefficient, DemandType.ArbitraryFeedForward, cosineScalar * 0.05);
    }

    public void setElbowVelocity(double deg){
        double radians = java.lang.Math.toRadians(getElbowPosition());
        double cosineScalar = java.lang.Math.cos(radians);
        mElbow.set(ControlMode.Velocity, deg * angularVelocityCoefficient, DemandType.ArbitraryFeedForward, cosineScalar * 0.05);
    }

    public void setWristPosition(double pos){
        mWrist.set(ControlMode.Position, pos * 177.6 / 2 / Math.PI);
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
        mSolenoid.set(Value.kForward);
    }

    private int rotationToNativeUnits(double rotationRads){
        double motorRotations = rotationRads / (2 * Math.PI) * kShoulderGearRatio;
        int sensorCounts = (int)(motorRotations * kCountsPerRev);
        return sensorCounts;
      }
    
      private int velocityToNativeUnits(double velocityRadPerSecond){
        double motorRotationsPerSecond = velocityRadPerSecond / (2 * Math.PI) * kShoulderGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
        return sensorCountsPer100ms;
      }
    
      private double nativeUnitsToRotationRad(double sensorCounts){
        double motorRotations = (double)sensorCounts / kCountsPerRev;
        double jointRotations = motorRotations / kShoulderGearRatio;
        double positionRads = jointRotations * 2 * Math.PI;
        return positionRads;
      }
}
