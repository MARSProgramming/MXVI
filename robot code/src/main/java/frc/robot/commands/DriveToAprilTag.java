
package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToAprilTag extends CommandBase {
    /** Default constraints are 50% of max speed, accelerate to full speed in 1 second */
    private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
    DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 0.5,
    DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
    /** Default constraints are 40% of max speed, accelerate to full speed in 1 second */
    private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
    DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.4,
    DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private final AprilTagFieldLayout mField;

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, DEFAULT_XY_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, DEFAULT_XY_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, DEFAULT_OMEGA_CONSTRAINTS);

    public DriveToAprilTag(DrivetrainSubsystem subsystem) {
        mDrivetrainSubsystem = subsystem;
        mField = subsystem.getField();

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Optional<Pose3d> checkForPose = mField.getTagPose(mDrivetrainSubsystem.resetPoseToLimelight());
 
        Pose2d robotPose = mDrivetrainSubsystem.getPose();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());

        if (checkForPose.isPresent()) {
            Pose3d goalPose = checkForPose.get();
            xController.setGoal(goalPose.getX()+0.5);
            yController.setGoal(goalPose.getY());
            if (robotPose.getRotation().getRadians()<0) {
                omegaController.setGoal(-Math.PI);
                SmartDashboard.putNumber("desiredrot", -Math.PI);    
            }
            else {
                omegaController.setGoal(Math.PI);
                SmartDashboard.putNumber("desiredrot", Math.PI);    
            }
            SmartDashboard.putNumber("desiredX", goalPose.getX()+1);
            SmartDashboard.putNumber("desiredY", goalPose.getY());
        }
        else {
            xController.setGoal(robotPose.getX());
            yController.setGoal(robotPose.getY());
            omegaController.setGoal(robotPose.getRotation().getRadians());
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d robotPose = mDrivetrainSubsystem.getPose();
        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }
        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }
        mDrivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDrivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, mDrivetrainSubsystem.getPose().getRotation()));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }
}
