package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Limelight extends SubsystemBase implements Loggable{
    @Log
    double[] botpose;
    public Limelight(){}

    @Override
    public void periodic(){
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false)){
            botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
            double x = botpose[0];
            double y = botpose[1];
            double z = botpose[2];
            DrivetrainSubsystem.getInstance().addVisionMeasurement(new Pose2d(x, y, new Rotation2d(botpose[3], botpose[4])));
        }
    }
}
