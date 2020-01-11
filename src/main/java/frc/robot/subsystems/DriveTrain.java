package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;

public class DriveTrain extends SubsystemBase {

    protected SpeedController m_frontLeft = new PWMTalonSRX(DriveConstants.mFrontLeft);
    protected SpeedController m_rearLeft = new PWMTalonSRX(DriveConstants.mRearLeft);
    protected SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

    protected SpeedController m_frontRight = new PWMTalonSRX(DriveConstants.mFrontRight);
    protected SpeedController m_rearRight = new PWMTalonSRX(DriveConstants.mRearRight);
    protected SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_frontRight, m_rearRight);

    protected Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    protected Encoder m_rightEncoder  = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);

    /* NAVX is not support for simulation so when real tests are run switch to navX and for simulation use AnalogGyro */
    //private final AHRS m_gyro = new AHRS(SPI.Port.kMXP); // NAVX
    private final AnalogGyro m_gyro = new AnalogGyro(DriveConstants.kGyroPort);

    private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftGroup, m_rightGroup);

    private final DifferentialDriveOdometry m_odometry;

    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

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

    public void tankDrive(final double leftSpeed, final double rightSpeed, final boolean squaredInputs) {
        m_differentialDrive.tankDrive(leftSpeed, rightSpeed, squaredInputs);
    }

    public void tankDrive(final double leftSpeed, final double rightSpeed) {
        tankDrive(leftSpeed, rightSpeed, true);
    }

    public void tankDrive(final double speed) {
        tankDrive(speed, speed);
    }

    public void stopTankDrive() {
        tankDrive(0);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftGroup.setVoltage(leftVolts);
        m_rightGroup.setVoltage(-rightVolts);
      }

    public void setMaxOutput(final double maxOutput) {
        m_differentialDrive.setMaxOutput(maxOutput);
    }

    /**
     * Methods for Encoder Data
     **/

    public void setDistancePerPulse() {
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    }

    public double getLeftEncoder() {
        return m_leftEncoder.getDistance();
    }

    public double getRightEncoder() {
        return m_rightEncoder.getDistance();
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
        return (getLeftEncoder() + getRightEncoder()) / 2;
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

    /** Methods for Kinematics */

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(final DifferentialDriveWheelSpeeds speeds) {
        double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(),
        speeds.leftMetersPerSecond);
        
        double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(),
        speeds.rightMetersPerSecond);

        m_leftGroup.set(leftOutput);
        m_rightGroup.set(rightOutput);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
      }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot    Angular velocity in rad/s.
     */
    @SuppressWarnings("ParameterName")
    public void drive(final double xSpeed, final double rot) {
        final DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
  }

  /**
   * Odometry and Kinematics
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
