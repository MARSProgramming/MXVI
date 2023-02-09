package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    private WPI_TalonFX mElbow = new WPI_TalonFX(Constants.Arm.kElbowMotorID);
    private WPI_TalonFX mShoulder = new WPI_TalonFX(Constants.Arm.kShoulderMotorID);

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (2048 pulses)
  private static final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / kCountsPerRev;
  final int k100msPerSecond = 10;

  /* Object for simulated inputs into Talon. */
//  TalonFXSimCollection m_ElbowSim = m_Elbow.getSimCollection();
  TalonFXSimCollection m_ShoulderSim = mShoulder.getSimCollection();
  TalonFXSimCollection m_ElbowSim = mElbow.getSimCollection();

  private static final double kShoulderGearRatio = 53; //Gear ratio for both joints is 53:1
  private static final double kUpperArmMass = 3.19; //Kilograms. TODO: what is the mass of upper arm in kg?
  private static final double kUpperArmLength = Units.inchesToMeters(23); //TODO: what is the length of the upper arm in inches?
  private static final double kElbowGearRatio = 53; //Gear ratio for both joints is 53:1
  private static final double kLowerArmMass = 2.49; //Kilograms. TODO: what is the mass of lower arm in kg?
  private static final double kLowerArmLength = Units.inchesToMeters(23); //TODO: what is the length of the lower arm in inches?
  private static final double kWristMass = 1.75; // Kilograms. TODO: what is the mass of wrist in kg?
  private static final double kWristLength = Units.inchesToMeters(8); //TODO: what is the length of the wrist?

  private static final int k_upper_arm_min_angle = -36000000; 
  private static final int k_upper_arm_max_angle = 36000000; 
  private static final int k_lower_arm_min_angle = -36000000; 
  private static final int k_lower_arm_max_angle = 36000000; 

  SingleJointedArmSim m_upperArmSim = new SingleJointedArmSim(
    DCMotor.getFalcon500(1),  //1 Falcon 500 controls the upper arm.
    kShoulderGearRatio,
    SingleJointedArmSim.estimateMOI(kUpperArmLength, kUpperArmMass),
    kUpperArmLength,
    Units.degreesToRadians(k_upper_arm_min_angle),
    Units.degreesToRadians(k_upper_arm_max_angle),
    kUpperArmMass,
    false,
    VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );

  SingleJointedArmSim m_lowerArmSim = new SingleJointedArmSim(
    DCMotor.getFalcon500(1),  //1 Falcon 500 controls the upper arm.
    kElbowGearRatio,
    SingleJointedArmSim.estimateMOI(kLowerArmLength + kWristLength, kUpperArmMass+kWristMass),
    kLowerArmLength+kWristLength,
    Units.degreesToRadians(k_lower_arm_min_angle),
    Units.degreesToRadians(k_lower_arm_max_angle),
    kLowerArmMass+kWristMass,
    false,
    VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving double jointed Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armShoulder = m_mech2d.getRoot("ArmShoulder", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armShoulder.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_upperArm =
      m_armShoulder.append(
          new MechanismLigament2d(
              "UpperArm",
              23,
              Units.radiansToDegrees(m_upperArmSim.getAngleRads()),
              6, 
              new Color8Bit(Color.kYellow)));
  private final MechanismLigament2d m_lowerArm =
      m_upperArm.append(
          new MechanismLigament2d(
              "LowerArm",
              23,
              Units.radiansToDegrees(m_lowerArmSim.getAngleRads()),
              10,
              new Color8Bit(Color.kPurple)));
    public Arm(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        mElbow.configAllSettings(config);
        mShoulder.configAllSettings(config);

        mElbow.setNeutralMode(NeutralMode.Brake);
        mShoulder.setNeutralMode(NeutralMode.Brake);

        mElbow.config_kP(0, 0.1);
        mElbow.config_kI(0, 0.0003);

        mShoulder.config_kP(0, 0.03);
        mShoulder.config_kI(0, 0.0001);

        SmartDashboard.putData("Arm Sim", m_mech2d);
        m_armTower.setColor(new Color8Bit(Color.kBlue));    
    }

    public void runElbowPOutput(double v){
        mElbow.set(ControlMode.PercentOutput, v);
    }

    public void runShoulderPOutput(double v){
        mShoulder.set(ControlMode.PercentOutput, v);
    }

    @Override
    public void periodic() {
      /*
       * This will get the simulated sensor readings that we set
       * in the previous article while in simulation, but will use
       * real values on the robot itself.
       */
      SmartDashboard.putNumber("Shoulder Position (ticks)", mShoulder.getSelectedSensorPosition());
      SmartDashboard.putNumber("Shoulder Position (deg)", Units.radiansToDegrees(nativeUnitsToRotationRad(mShoulder.getSelectedSensorPosition())));
  
      SmartDashboard.putNumber("Elbow Position (ticks)", mElbow.getSelectedSensorPosition());
      SmartDashboard.putNumber("Elbow Position (deg)", Units.radiansToDegrees(nativeUnitsToRotationRad(mElbow.getSelectedSensorPosition())));
    }

    public double getShoulderPosition(){
      return Units.radiansToDegrees(nativeUnitsToRotationRad(mShoulder.getSelectedSensorPosition()));
    }

    public double getElbowPosition(){
        return Units.radiansToDegrees(nativeUnitsToRotationRad(mElbow.getSelectedSensorPosition()));
    }

    public void goToAngles(double a1, double a2){
      mElbow.set(ControlMode.Position, rotationToNativeUnits(a1));
      mShoulder.set(ControlMode.Position, rotationToNativeUnits(a2));
    }

    public void goToPoint(){
      double x = 0.0;
      double y = 0.3;
      double theta1 = 0;
      double theta2 = 0;

      if (Math.pow(x,2) + Math.pow(y,2) > Math.pow((kLowerArmLength + kUpperArmLength), 2)){
        theta1 = Math.toRadians(getShoulderPosition());
        theta2 = Math.toRadians(getElbowPosition());
      }
      else{
        double c2 = (Math.pow(x,2) + Math.pow(y,2) - Math.pow(kLowerArmLength,2) - Math.pow(kUpperArmLength,2)) / (2 * kLowerArmLength * kUpperArmLength);
        double s2 = Math.sqrt(1 - Math.pow(c2,2));
        theta2 = Math.atan2(s2, c2);
        double k1 = kUpperArmLength + kLowerArmLength * c2;
        double k2 = kLowerArmLength * s2;
        theta1 = Math.atan2(y, x) - Math.atan2(k2, k1);
      }
      goToAngles(theta2, theta1);
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
      Xd.set(0, 0, xdot);
      Xd.set(1, 0, ydot);

      System.out.println(A.invert());
      SimpleMatrix B = A.invert().mult(Xd);
      System.out.println(B);
      mShoulder.set(ControlMode.PercentOutput, -B.get(0, 0));
      mElbow.set(ControlMode.PercentOutput, -B.get(1, 0));
      SmartDashboard.putNumber("B", Units.radiansToDegrees(-B.get(0, 0)));
      SmartDashboard.putNumber("A", Units.radiansToDegrees(-B.get(1, 0)));
      //System.out.println("B " + -B.get(0, 0) + " " + -B.get(1, 0));
    }

    @Override
    public void simulationPeriodic() {
      /* Pass the robot battery voltage to the simulated Talon FXs */
      m_ElbowSim.setBusVoltage(RobotController.getBatteryVoltage());
      m_ShoulderSim.setBusVoltage(RobotController.getBatteryVoltage());
  
      /*
       * WPILib expects +V to be forward.
       * Positive motor output lead voltage is ccw. 
       */
      m_upperArmSim.setInput(m_ShoulderSim.getMotorOutputLeadVoltage());
      m_lowerArmSim.setInput(m_ElbowSim.getMotorOutputLeadVoltage());
  
      /*
       * Advance the model by 20 ms. Note that if you are running this
       * subsystem in a separate thread or have changed the nominal
       * timestep of TimedRobot, this value needs to match it.
       */
      m_upperArmSim.update(0.02);
      m_lowerArmSim.update(0.02);
  
      /*
       * Update all of our sensors.
       * WPILib's simulation class is assuming +V is forward
       */
      
      m_ShoulderSim.setIntegratedSensorRawPosition(
                      rotationToNativeUnits(
                          m_upperArmSim.getAngleRads()
                      ));
      m_ShoulderSim.setIntegratedSensorVelocity(
                      velocityToNativeUnits(
                          m_upperArmSim.getVelocityRadPerSec()
                      ));
      m_ElbowSim.setIntegratedSensorRawPosition(
                      rotationToNativeUnits(
                          m_lowerArmSim.getAngleRads()
                      ));
      m_ElbowSim.setIntegratedSensorVelocity(
                      velocityToNativeUnits(
                          m_lowerArmSim.getVelocityRadPerSec()
                      ));
        // SimBattery estimates loaded battery voltages
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(m_upperArmSim.getCurrentDrawAmps()));
  
      // Update the Mechanism Arm angle based on the simulated arm angle
      m_upperArm.setAngle(Units.radiansToDegrees(m_upperArmSim.getAngleRads()));
      m_lowerArm.setAngle(Units.radiansToDegrees(m_lowerArmSim.getAngleRads()));
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

