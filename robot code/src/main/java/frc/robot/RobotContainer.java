// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.Intake;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.commands.ZeroSwerves;
import frc.robot.commands.Arm.ManualElbowLeft;
import frc.robot.commands.Arm.ManualElbowRight;
import frc.robot.commands.Arm.ManualShoulderLeft;
import frc.robot.commands.Arm.ManualShoulderRight;
import frc.robot.commands.Arm.ManualWrist;
import frc.robot.commands.Arm.Paths.MoveVelocity;
import frc.robot.commands.Arm.Paths.StartToLoad;
import frc.robot.commands.Arm.Paths.StartToScoreLeft;
import frc.robot.commands.Arm.Paths.StartToScoreRight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BottomSolenoids;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Paddle;
import frc.robot.util.AutoChooser;
import frc.robot.util.CustomXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();

  private final CustomXboxController mPilot = new CustomXboxController(0);
  private final CustomXboxController mCoPilot = new CustomXboxController(1);

  private HashMap<String, Pose2d> mPointPositionMap;
  private AutoChooser autoChooser = new AutoChooser(mDrivetrainSubsystem);

  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final BottomSolenoids mBottomSolenoids = new BottomSolenoids();
  private final Arm mArm = new Arm();
  private final Paddle mPaddle = new Paddle();

  private final Compressor mCompressor = new Compressor(61, PneumaticsModuleType.REVPH);

  public double getPressure(){
    return mCompressor.getPressure();
  }

  public void startCompressor(){
    mCompressor.enableAnalog(100, 110);
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    /*mDrivetrainSubsystem.setDefaultCommand(new DriveSnapRotation(
            mDrivetrainSubsystem,
            () -> -modifyAxis(mPilot.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> -modifyAxis(mPilot.getRightY()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            mDrivetrainSubsystem.getSnapController()
    ));*/
    mDrivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            mDrivetrainSubsystem,
            () -> -modifyAxis(mPilot.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    SmartDashboard.putData("Zero Swerves", new ZeroSwerves(mDrivetrainSubsystem).withTimeout(1).ignoringDisable(true));
    SmartDashboard.putData(CommandScheduler.getInstance());
    mPointPositionMap = new HashMap<>();
    mPointPositionMap.put("A", new Pose2d(0, 0, new Rotation2d(Math.toRadians(0.0))));
    configureTeleopBindings();
  }

  public void initializeSolenoids(){
    mIntakeSubsystem.initializeSolenoid();
    mPaddle.initializeSolenoid();
    mArm.initializeSolenoid();
    mBottomSolenoids.initializeSolenoid();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureTeleopBindings() {
    //mCoPilot.getRightDPadObject().whileTrue(new MoveVelocity(mArm, () -> deadband(mCoPilot.getLeftX(), 0.2)/1000, () -> deadband(mCoPilot.getLeftY(), 0.2)/1000));
    mPilot.getYButtonObject().onTrue(new ZeroGyroscope(mDrivetrainSubsystem, 0));
    mPilot.getLeftTriggerObject().whileTrue(new Intake(mIntakeSubsystem));

    //Claw solenoid command(s)
    mCoPilot.getAButtonObject().onTrue(mArm.toggleClaw());

    //Bottom Solenoid command(s)
    //mCoPilot.getXButtonObject().onTrue(mBottomSolenoids.toggleBottomSolenoid());

    //Paddle command(s)
    //mCoPilot.getBButtonObject().onTrue(mPaddle.togglePaddle());

    //Intake command(s)
    mPilot.getAButtonObject().onTrue(mIntakeSubsystem.toggleIntake());

    mCoPilot.getLeftTriggerObject().whileTrue(new ManualElbowLeft(mArm));
    mCoPilot.getRightTriggerObject().whileTrue(new ManualElbowRight(mArm));
    mCoPilot.getLeftBumperObject().whileTrue(new ManualShoulderLeft(mArm));
    mCoPilot.getRightBumperObject().whileTrue(new ManualShoulderRight(mArm));
    //mCoPilot.getXButtonObject().whileTrue(new MoveWrist(mArm, 0));
    //mCoPilot.getBButtonObject().whileTrue(new MoveWrist(mArm, Math.PI));
    mCoPilot.getUpDPadObject().whileTrue(new ManualWrist(mArm));
    //mCoPilot.getBButtonObject().whileTrue(new GoToAngles(mArm, Math.PI/2, 0.0));
    mCoPilot.getYButtonObject().whileTrue(new StartToLoad(mArm));
    mCoPilot.getXButtonObject().whileTrue(new StartToScoreRight(mArm));
    //new Trigger(() -> mPilot.getLeftTriggerAxis() > 0.2).onTrue(mIntakeSubsystem.runIntakeMotors(() -> mPilot.getRightTriggerAxis()));
   // mPilot.getRightTriggerObject().onTrue(new Intake(mIntakeSubsystem));

    System.out.println("Teleop Bindings Configured");
  }

  public void configureTestBindings(){
    
    //Claw solenoid command(s)
    mPilot.getYButtonObject().onTrue(mArm.toggleClaw());

    //Bottom Solenoid command(s)
    mPilot.getXButtonObject().onTrue(mBottomSolenoids.toggleBottomSolenoid());

    //Paddle command(s)
    mPilot.getBButtonObject().onTrue(mPaddle.togglePaddle());

    //Intake command(s)
    mPilot.getAButtonObject().onTrue(mIntakeSubsystem.toggleIntake());
    //new Trigger(() -> mPilot.getLeftTriggerAxis() > 0.2).onTrue(mIntakeSubsystem.runIntakeMotors(() -> mPilot.getRightTriggerAxis()));
    System.out.println("Test Bindings Configured");
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);

      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    //"Stage" mode
    //value = Math.round(value * 5.0)/5.0;

    return value;
  }

  public XboxController getPilot(){
    return mPilot;
  }
}
