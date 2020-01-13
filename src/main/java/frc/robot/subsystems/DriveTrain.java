package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.controller.PIDController;

public class DriveTrain extends SubsystemBase {

    protected TalonSRX m_frontLeft = new TalonSRX(DriveConstants.mFrontLeft);
    protected TalonSRX m_rearLeft = new TalonSRX(DriveConstants.mRearLeft);

    protected TalonSRX m_frontRight = new TalonSRX(DriveConstants.mFrontRight);
    protected TalonSRX m_rearRight = new TalonSRX(DriveConstants.mRearRight);

    protected Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    protected Encoder m_rightEncoder  = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP); // NAVX

    private final DifferentialDriveOdometry m_odometry;

    public EncoderFollower m_left_follower;
    public EncoderFollower m_right_follower;

    public Notifier m_follower_notifier;

    public DriveTrain() {

        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        m_leftEncoder.reset();
        m_rightEncoder.reset();

        m_odometry = new DifferentialDriveOdometry(getHeading(), DriveConstants.startingPosition);
    }

    @Override
    public void periodic() {
      // Update the odometry in the periodic block
      updateOdometry();
    }

    /**
     * Drive Methods
     */

    public void tankDrive(final double leftSpeed, final double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
    }

    public void tankDriveSquared(final double leftSpeed, final double rightSpeed) {
            tankDrive(leftSpeed * leftSpeed,rightSpeed * rightSpeed);
    }

    public void tankDrive(final double speed) {
        tankDrive(speed, speed);
    }

    public void setRightSpeed(double speed) {
        m_frontRight.set(ControlMode.PercentOutput, speed);
        m_rearRight.set(ControlMode.PercentOutput, speed);
    }

    public void setLeftSpeed(double speed) {
        m_frontLeft.set(ControlMode.PercentOutput, speed);
        m_rearLeft.set(ControlMode.PercentOutput, speed);
    }

    public void stopTankDrive() {
        tankDrive(0);
    }

    /**
     * Methods for Encoder Data
     **/

    public void setDistancePerPulse() {
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    }

    public double getLeftEncoderDistance() {
        return m_leftEncoder.getDistance();
    }

    public double getRightEncoderDistance() {
        return m_rightEncoder.getDistance();
    }

    public int getRightEncoder() {
        return m_rightEncoder.get();
    }

    public int getLeftEncoder() {
        return m_leftEncoder.get();
    }

    public void resetLeftEncoder() {
        m_leftEncoder.reset();
    }

    public void resetRightEncoder() {
        m_rightEncoder.reset();
    }

    public void resetEncoders() {
        resetRightEncoder();
        resetLeftEncoder();
    }


    public double getDistanceTraveled() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;
    }

    /**
     * Methods for Gyro Data
     * 
     * @return The displacement in degrees from -180 to 180
     */


    public double getAngle() {
        return m_gyro.getAngle();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public void resetGryo() {
        m_gyro.reset();
    }


  /**
   * Odometry
   */

   public Pose2d getPose2d() {
       return m_odometry.getPoseMeters();
   }

   public Translation2d getTranslation2d() {
       return getPose2d().getTranslation();
   }

   public double getX() {
       return getTranslation2d().getX();
   }

   public double getY() {
    return getTranslation2d().getY();
}

  public void updateOdometry() {
    m_odometry.update(getHeading(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getHeading());
  }
}
