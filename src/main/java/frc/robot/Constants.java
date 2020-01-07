/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
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

		// Encoder Ports
		public static final int eLeftA = 2;
		public static final int eLeftB = 3;
		public static final int eRightA = 0;
		public static final int eRightB = 1;

		// Gyro
		public static final int gDrive = 1;

		// Constants
		public static final double kMaxSpeed = 3.0; // meters per second
		public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
	  
		public static final double kTrackWidth = 0.381 * 2; // meters
		public static final double kWheelRadius = 0.0508; // meters
		public static final int kEncoderResolution = 4096;
	}
}
