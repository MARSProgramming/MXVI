// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
/**
 * The left-to-right distance between the drivetrain wheels
 *
 * Should be measured from center to center.
 */

        public static class Drive{
            public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57;
            public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57;
            public static final double MAX_SPEED_MULTIPLIER = 0.4; 

            public static final int DRIVETRAIN_PIGEON_ID = 31; 
            public static final String kDriveCANivore = "Drivetrain";

                
            public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 15; 
            public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5; 
            public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 25; 

            public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 14; 
            public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; 
            public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 24;  

            public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 16; 
            public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
            public static final int BACK_LEFT_MODULE_STEER_ENCODER = 26; 

            public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 17; 
            public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; 
            public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 27; 

            public static final double kP = 0.3;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
            public static double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
            public static double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
            public static double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
        }

    public static class Auto{
        public static final double holonomicXkP = 1.0;
        public static final double holonomicXkI = 0;
        public static final double holonomicXkD = 0;
        public static final double holonomicYkP = 1.0;
        public static final double holonomicYkI = 0;
        public static final double holonomicYkD = 0;
        public static final double holonomicOkP = 2.5;
        public static final double holonomicOkI = 0.0;
        public static final double holonomicOkD = 0.0;
        public static final double holonomicOMaxVelocity = 2;
        public static final double holonomicOMaxAcceleration = 5;
    }

    public static class Controller{
        public static final double kTriggerThreshold = 0.6;
    }

    public static class Arm{
        public static final int kElbowMotorID = -1;
        public static final int kShoulderMotorID = -1;
    }
}
