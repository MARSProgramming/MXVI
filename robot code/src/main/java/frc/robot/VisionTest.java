package frc.robot;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;

public class VisionTest {
    
    //Instantiating photo camera with name 
    PhotonCamera camera = new PhotonCamera("photonvision");
    
    //Getting the latest result from photonvision (contains all of the camera info)
    PhotonPipelineResult results = camera.getLatestResult(); 

    //Separating all of the relevant information that we would use: 


    boolean hasTarget = results.hasTargets(); 

    double latency = results.getLatencyMillis(); //Latency of the camera in ms

    /*
     This gives you the best target, so that you can get the needed information from just htat target
     Best target is defined by largest, smallest, highest, lowest, rightmost, leftmost, or centermost based on what was selected in camera settings, can be changed from limelight
     dashboard 
    */
    PhotonTrackedTarget target = results.getBestTarget(); 

    double yaw = target.getYaw(); // Yaw of the target in degrees (positive is to the right)
    double pitch = target.getPitch();  // Pitch of the target in degrees (positive is up)
    double skew = target.getSkew(); //Skew of the target in degrees (counter-clockwise is positive)

    double area =  target.getArea(); // Area of the target as a percent (0-100); how much of the camera feed is taken up by the target box

    //All of the above work for AprilTage except for getSkew(); 

    // Getting AprilTag specific data from the target: 

    int fiducialID = target.getFiducialId(); //This gives the fiducial ID of the apriltag, or the ID it has been given (typically a number)
    double ambiguity = target.getPoseAmbiguity(); //This gives the ambiguity for the pose of the target (how much estimated error there is for our information)


    // Using available information to calculate distance to the target (assumes fixed camera height and fixed target height): 

    double cameraHeight = 0; //Put in camera height here in meters
    double targetHeight = 0; //Put in target height here in meters
    double cameraPitch =  0; //Put in the pitch of the camera here in RADIANS
    double range = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitch, Units.degreesToRadians(results.getBestTarget().getPitch()));
}
