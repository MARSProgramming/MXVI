package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveSnapRotation extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationXSupplier;
    private final DoubleSupplier m_rotationYSupplier;
    private ProfiledPIDController mSnapController;

    public DriveSnapRotation(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationXSupplier, DoubleSupplier rotationYSupplier, ProfiledPIDController snapController) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationXSupplier = rotationXSupplier;
        this.m_rotationYSupplier = rotationYSupplier;
        mSnapController = snapController;

        mSnapController.setGoal(new TrapezoidProfile.State(Math.atan2(m_rotationYSupplier.getAsDouble(), m_rotationXSupplier.getAsDouble()), 0.0));
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        SmartDashboard.putNumber("desiredSnap", Math.atan2(-m_rotationYSupplier.getAsDouble(), m_rotationXSupplier.getAsDouble()));
        double angleAdjustment = mSnapController.calculate(m_drivetrainSubsystem.getPigeonAngle(), Math.atan2(-m_rotationYSupplier.getAsDouble(), m_rotationXSupplier.getAsDouble())+Math.PI/2);
        m_drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_translationXSupplier.getAsDouble(),
                    m_translationYSupplier.getAsDouble(),
                    Math.abs(m_rotationYSupplier.getAsDouble()) > 0 || Math.abs(m_rotationXSupplier.getAsDouble()) > 0 ? angleAdjustment * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND : 0,
                    m_drivetrainSubsystem.getGyroscopeRotation()
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
