package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;

public class DriveTrain extends SubsystemBase {

    private static DriveTrain m_instance;

    private final WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(DriveConstants.kFrontLeftPort);
    private final WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(DriveConstants.kRearLeftPort);
    private final SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

    private final WPI_TalonSRX m_frontRight = new WPI_TalonSRX(DriveConstants.kFrontRightPort);
    private final WPI_TalonSRX m_rearRight = new WPI_TalonSRX(DriveConstants.kRearRightPort);
    private final SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);


    private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    private final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);

    private AHRS m_ahrs; // NAVX

    private final DifferentialDriveOdometry m_odometry;

    private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_left, m_right);

    public DriveTrain() {

        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        m_leftEncoder.reset();
        m_rightEncoder.reset();

        try {
            m_ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error installing navX MXP: " + ex.getMessage(), true);
        }
        resetGryo();

        m_odometry = new DifferentialDriveOdometry(getHeading(), AutoConstants.kStartingPosition);
        m_differentialDrive.setSafetyEnabled(false);
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    public static DriveTrain getInstance() {
        if (m_instance == null) {
            m_instance = new DriveTrain();
        }

        return m_instance;
    }

    /**
     * Drive Methods
     * */

    public void tankDrive(final double leftSpeed, final double rightSpeed, final boolean squaredInputs) {
        m_differentialDrive.tankDrive(leftSpeed, rightSpeed, squaredInputs);
    }

    public void tankDrive(final double leftSpeed, final double rightSpeed) {
        tankDrive(leftSpeed, rightSpeed, false);
    }

    public void tankDrive(final double speed) {
        tankDrive(speed, speed);
    }

    public void stopTankDrive() {
        tankDrive(0);
    }

    public void tankDriveVolts(final double leftVolts, final double rightVolts) {
        m_left.setVoltage(leftVolts);
        m_right.setVoltage(rightVolts);
    }

    public void setMaxOutput(double maxOutput) {
        m_differentialDrive.setMaxOutput(maxOutput);
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

    public int getRightEncoderValue() {
        return m_rightEncoder.get();
    }

    public int getLeftEncoderValue() {
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

    public double getYaw() {
        return m_ahrs.getYaw();
    }

    public double getAngle() {
        return m_ahrs.getAngle(); // Same as yaw
    }

    public double getPitch() {
        return m_ahrs.getPitch();
    }

    public double getRoll() {
        return m_ahrs.getRoll();
    }

    public double getAccelerationX() {
        return m_ahrs.getRawAccelX();
    }

    public double getAccelerationY() {
        return m_ahrs.getRawAccelY();
    }

    public double getAccelerationZ() {
        return m_ahrs.getRawAccelZ();
    }

    public void resetGryo() {
        m_ahrs.reset();
    }

    public boolean isCalibrating() {
        return m_ahrs.isCalibrating();
    }

    public double getTurnRate() {
        return m_ahrs.getRate(); // Returns yaw rate
    }

    /**
     * Odometry methods
     * */

    public Pose2d getPose2d() {
        return m_odometry.getPoseMeters();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getAngle());
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

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    public void updateOdometry() {
        m_odometry.update(getHeading(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, getHeading());
    }
}