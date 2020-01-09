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
	public static final class OIConstants {
		//Stick Ports
		public static int jLeft = 0;
		public static int jRight = 1;
	}

	public static final class DriveConstants {
		// Motor Ports
		public static final int mFrontLeft = 5;
		public static final int mRearLeft = 6;

		public static final int mFrontRight = 1;
		public static final int mRearRight = 2;

		// Gyro
		public static final int kGyroPort = 0; // Placeholder that is subject to change

		// Encoders
		public static final int[] kLeftEncoderPorts = new int[]{0, 1};
		public static final int[] kRightEncoderPorts = new int[]{2, 3};

		public static final boolean kLeftEncoderReversed = false;
		public static final boolean kRightEncoderReversed = true;

		public static final int kEncoderCPR = 1024; // Placeholder that is subject to change
		public static final double kWheelDiameterMeters = 0.15; // Placeholder that is subject to change
		public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
	
		// Kinematics
		public static final double kTrackwidthMeters = 0.69; // Placeholder that is subject to change
		
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

		// Odometry
		public static final Pose2d startingPosition = new Pose2d(new Translation2d(0, 0), new Rotation2d(0)); // Placeholder that is subject to change

		public static final double fieldLengthMeters = 15.98;
		public static final double fieldHeightMeters = 8.21;
	}

	public static final class LiftConstants {
		public static int liftSolenoidA = 0; // not configured yet lift system might change
		public static int liftSolenoidB = 0; // not configured yet lift system might change
	}

	public static final class TurretConstants {
		// Motor
		public static final int kMotorPort = 7; // Placeholder that is subject to change
		
		// Gyro
		public static final int kGyroPort = 1; // Placeholder that is subject to change

		// PID
		public static final double kP = 0; // Placeholder that is subject to change
		public static final double kI = 0; // Placeholder that is subject to change
		public static final double kD = 0; // Placeholder that is subject to change

		// Constraints
		public static final double kMaxVel = 1.75; // Placeholder that is subject to change
		public static final double kMaxAcc = 0.75; // Placeholder that is subject to change

		public static final Translation2d goalPosition = new Translation2d(4, 3); // Placeholder that is subject to change
	}

	public static final class MiscellaneousConstants {
		public static final double kDt = 0.02; 
	}
}
