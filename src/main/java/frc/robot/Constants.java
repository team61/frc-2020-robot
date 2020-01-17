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
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

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
        public static int jTurretHeading = 2;
        public static int jTurretAngle = 3;
    }

    public static final class DriveConstants {

        // Robot Infomation
        public static final double kTrackwidth = 0.69; // Meters
        public static final double kTrackheight = 0.94; // Meters

        public static final double kWheelDiameter = 0.15; // Meters

        public static final double kMaxVelocity = 5; // Meters per second

        public static final double kMaxAcceleration = 3; // Meters per second squared

        // Encoder Infomation
        public static final int kEncoderCPR = 1024; // Placeholder that is subject to change
        public static final double kEncoderDistancePerPulse = (kWheelDiameter * Math.PI) / (double) kEncoderCPR;
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        // Kinematics
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);

        // Odometry
        public static final Pose2d kStartingPosition = new Pose2d(new Translation2d(0, 0), new Rotation2d(0)); // Placeholder that is subject to change

        // Motor Ports
        public static final int kLeftMasterPort = 5;
        public static final int kLeftSlavePort = 6;
        public static final int kRightMasterPort = 1;
        public static final int kRightSlavePort = 2;

        // Encoder Ports
        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
    }

    public static final class TurretConstants {
        // Motor
        public static final int kHeadingMotorPort = 7; // Placeholder that is subject to change
        public static final int kAngleMotorPort = 11; // Placeholder

        public static final Translation2d goalPosition = new Translation2d(-4, 4); // Placeholder that is subject to change
        public static final double goalHeight = 20; // placeholder that is subject to change

        public static final double kDistanceFromRobot = 0.3;

        public static final double kAngleFromRobot = 0;

        public static final class HeaderConstants {
            // PID
            public static final double kP = 0; // Placeholder that is subject to change
            public static final double kI = 0; // Placeholder that is subject to change
            public static final double kD = 0; // Placeholder that is subject to change

            // Constraints
            public static final double kMaxVel = 1.75; // Placeholder that is subject to change
            public static final double kMaxAcc = 0.75; // Placeholder that is subject to change

            public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVel, kMaxAcc);

            public static final double kRange = 180; // in degrees
            public static final double kUpperLimit = kRange/2;
            public static final double kLowerLimit = -kRange/2;
        }

        public static final class AngleConstants {
            // PID
            public static final double kP = 0; // Placeholder that is subject to change
            public static final double kI = 0; // Placeholder that is subject to change
            public static final double kD = 0; // Placeholder that is subject to change

            // Constraints
            public static final double kMaxVel = 1.75; // Placeholder that is subject to change
            public static final double kMaxAcc = 0.75; // Placeholder that is subject to change

            public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVel, kMaxAcc);

            public static final double kRange = 90; // in degrees
            public static final double kUpperLimit = kRange;
            public static final double kLowerLimit = 0;
        }
    }

    public static final class IntakeConstants {
        public static final int kMotorPort = 8; // Placeholder

        public static final double kIntakeSpeedPer = 0.5;
        // Constraints
        public static final double kMaxVel = 1.75; // Placeholder that is subject to change
        public static final double kMaxAcc = 0.75; // Placeholder that is subject to change

        public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVel, kMaxAcc);
    }

    public static final class FeederConstants {
        public static final int kMotorPort = 9; // Placeholder
        public static final int kLimitSwitchPort = 0; // Placeholder

        public static final double kFeederSpeedPer = 0.5;
        // Constraints
        public static final double kMaxVel = 1.75; // Placeholder that is subject to change
        public static final double kMaxAcc = 0.75; // Placeholder that is subject to change

        public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVel, kMaxAcc);
    }

    public static final class LauncherConstants {
        public static final int kFlywheelMotorAPort = 10; // Placeholder
        public static final int kFlywheelMotorBPort = 11; // Placeholder

        public static final int kLimitSwitchPort = 1; // Placeholder

        public static final double kFastSpeed = 0.5; // Placeholder
        public static final double kSlowSpeed = 1; // Placeholder

        // Constraints
        public static final double kMaxVel = 1.75; // Placeholder that is subject to change
        public static final double kMaxAcc = 0.75; // Placeholder that is subject to change

        // PID
        public static final double kP = 0; // Placeholder that is subject to change
        public static final double kI = 0; // Placeholder that is subject to change
        public static final double kD = 0; // Placeholder that is subject to change
        

        public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVel, kMaxAcc);
    }

    public static final class PhysicsConstants {
        public static final double kG = 9.81;
    }

    public static final class MiscellaneousConstants {
        public static final double kDt = 0.02;
    }
}
