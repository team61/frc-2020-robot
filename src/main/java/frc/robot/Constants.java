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
		public static final int gDrive = 0;

		public static final int[] kLeftEncoderPorts = new int[]{0, 1};
		public static final int[] kRightEncoderPorts = new int[]{2, 3};
		public static final boolean kLeftEncoderReversed = false;
		public static final boolean kRightEncoderReversed = true;
	
		public static final double kTrackwidthMeters = 0.69;
		public static final DifferentialDriveKinematics kDriveKinematics =
			new DifferentialDriveKinematics(kTrackwidthMeters);
	
		public static final int kEncoderCPR = 1024;
		public static final double kWheelDiameterMeters = 0.15;
		public static final double kEncoderDistancePerPulse =
			// Assumes the encoders are directly mounted on the wheel shafts
			(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

		public static final Pose2d startingPosition = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));

		public static final double fieldLengthMeters = 15.98;
		public static final double fieldHeightMeters = 8.21;
	}
}
