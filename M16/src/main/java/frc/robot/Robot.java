// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //Compressor mCompressor = new Compressor(63, PneumaticsModuleType.REVPH);
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Logger.configureLoggingAndConfig(m_robotContainer, false);
    //mCompressor.enableAnalog(100, 110);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
    //SmartDashboard.putNumber("psi", mCompressor.getPressure());
    //SmartDashboard.putNumber("current", mCompressor.getCurrent());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    LiveWindow.disableAllTelemetry();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  //Compressor comp = new Compressor(1, PneumaticsModuleType.REVPH);
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.configureTeleopBindings();
    srx.setNeutralMode(NeutralMode.Brake);
    srx2.setNeutralMode(NeutralMode.Brake);
    srx2.follow(srx);
    srx2.setInverted(true);
    srx.setInverted(true);
  }

  //TalonFX thej = new TalonFX(14, "Drivetrain");
  /** This function is called periodically during operator control. */
  TalonSRX srx = new TalonSRX(0);
  TalonSRX srx2 = new TalonSRX(13);
  @Override
  public void teleopPeriodic() {
    //thej.set(ControlMode.PercentOutput, 0.5);
    if(m_robotContainer.getPilot().getLeftTriggerAxis() > 0.2){
      srx.set(ControlMode.PercentOutput, m_robotContainer.getPilot().getLeftTriggerAxis());
    }
    else{
      srx.set(ControlMode.PercentOutput, 0.0);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}