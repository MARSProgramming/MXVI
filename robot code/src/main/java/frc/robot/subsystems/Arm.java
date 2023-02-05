package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Arm extends SubsystemBase{
    private WPI_TalonFX mElbow;
    private WPI_TalonFX mShoulder;

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
  private static final double kLowerArmMass = 3.19; //Kilograms. TODO: what is the mass of lower arm in kg?
  private static final double kLowerArmLength = Units.inchesToMeters(23); //TODO: what is the length of the lower arm in inches?
  private static final double kWristMass = 2.0; // Kilograms. TODO: what is the mass of wrist in kg?
  private static final double kWristLength = Units.inchesToMeters(8); //TODO: what is the length of the wrist?

  private static final int k_upper_arm_min_angle = -180; 
  private static final int k_upper_arm_max_angle = 260; 
  private static final int k_lower_arm_min_angle = -90; 
  private static final int k_lower_arm_max_angle = 190; 

  SingleJointedArmSim m_upperArmSim = new SingleJointedArmSim(
    DCMotor.getFalcon500(1),  //1 Falcon 500 controls the upper arm.
    kShoulderGearRatio,
    SingleJointedArmSim.estimateMOI(kUpperArmLength, kUpperArmMass),
    kUpperArmLength,
    Units.degreesToRadians(k_upper_arm_min_angle),
    Units.degreesToRadians(k_upper_arm_max_angle),
    kUpperArmMass,
    true,
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
    true,
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
              28.5,
              Units.radiansToDegrees(m_lowerArmSim.getAngleRads()),
              10,
              new Color8Bit(Color.kPurple)));
  private final MechanismLigament2d m_wrist =
      m_lowerArm.append(
          new MechanismLigament2d(
            "Wrist",
            Units.metersToInches(kWristLength),
            0,
            20,
            new Color8Bit(Color.kGray)));

    public Arm(){
        mElbow = new WPI_TalonFX(Constants.Arm.kElbowMotorID);
        mShoulder = new WPI_TalonFX(Constants.Arm.kShoulderMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        mElbow.configAllSettings(config);
        mShoulder.configAllSettings(config);

        mElbow.setNeutralMode(NeutralMode.Brake);
        mShoulder.setNeutralMode(NeutralMode.Brake);

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
