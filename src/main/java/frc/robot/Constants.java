/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    /* SI Units are used */

    public static final class OIConstants {

        //Stick Ports
        public static int jLeft = 0;
        public static int jRight = 1;
    }

    public static final class DriveConstants {

        public static final double kWheelDiameter = 0.15; // Meters

        // Encoder Information
        public static final int kEncoderCPR = 1024; // Placeholder that is subject to change
        public static final double kEncoderDistancePerPulse = (kWheelDiameter * Math.PI) / (double) kEncoderCPR;
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        // Motor Ports
        public static final int kFrontLeftPort = 5;
        public static final int kRearLeftPort = 6;
        public static final int kFrontRightPort = 1;
        public static final int kRearRightPort = 2;

        // Encoder Ports
        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
    }

    public static final class AutoConstants {
        // Robot Infomation
        public static final double kTrackwidth = 0.69; // Meters
        public static final double kWheelBase = 0.94; // Meters

        public static final double kMaxVelocity = 5; // Meters per second

        public static final double kMaxAcceleration = 3; // Meters per second squared

        public static final double kMaxVoltage = 10;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);


        // Characterization

        // Feedforward
        public static final double kS = 0; // Volts
        public static final double kV = 0; // Volts seconds per meters
        public static final double kA = 0; // Volts seconds per meters squared

        // Feedback
        public static final double kP = 0; // Volts seconds per meter

        // Ramsete Controller
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final Pose2d kStartingPosition = new Pose2d(new Translation2d(0, 0), new Rotation2d(0)); // Placeholder that is subject to change
    }
}
