package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

public class DriveSubsystem extends SubsystemBase {

    private static DriveSubsystem m_instance;

    private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(DriveConstants.kFrontLeftPort);
    private final WPI_TalonSRX m_leftSlave = new WPI_TalonSRX(DriveConstants.kRearLeftPort);

    private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(DriveConstants.kFrontRightPort);
    private final WPI_TalonSRX m_rightSlave = new WPI_TalonSRX(DriveConstants.kRearRightPort);


    private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    private final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);

    private AHRS m_ahrs; // NAVX

    private final DifferentialDriveOdometry m_odometry;

    private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

    private double x = 0;
    private double y = 0;

    public DriveSubsystem() {

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
        m_differentialDrive.setDeadband(0); // Dead band is done with joysticks directly

        m_rightMaster.setInverted(false);
        m_leftMaster.setInverted(true);
        m_rightSlave.setInverted(false);
        m_leftSlave.setInverted(true);

        m_leftSlave.follow(m_leftMaster);
        m_rightSlave.follow(m_rightMaster);
    }

    @Override
    public void periodic() {
        updateOdometry();
        x = getPose2d().getTranslation().getX();
        y = getPose2d().getTranslation().getY();
    }

    public static DriveSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new DriveSubsystem();
        }

        return m_instance;
    }

    /**
     * Drive Methods
     * */

    public void tankDrive(final double leftSpeed, final double rightSpeed, final boolean squaredInputs) {
       m_differentialDrive.tankDrive(leftSpeed, rightSpeed * AutoConstants.kFudgeFactor, squaredInputs);
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

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMaster.setVoltage(leftVolts);
        m_rightMaster.setVoltage(rightVolts * AutoConstants.kFudgeFactor);
    }

    public void tankDriveToVolts(double leftSpeed, double rightSpeed, final boolean squaredInputs) {
        if (squaredInputs) {
            leftSpeed = Math.copySign(leftSpeed * leftSpeed, leftSpeed);
            rightSpeed = Math.copySign(rightSpeed * rightSpeed, rightSpeed);
        }
        leftSpeed *= AutoConstants.kMaxVoltage;
        rightSpeed *= AutoConstants.kMaxVoltage;
        tankDriveVolts(leftSpeed, rightSpeed);
    }
    public void tankDriveToVolts(double leftSpeed, double rightSpeed) {
        tankDriveToVolts(leftSpeed, rightSpeed, true);
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

    public double getLeftEncoderRate() {
        return m_leftEncoder.getRate();
    }

    public double getRightEncoderRate() {
        return m_rightEncoder.getRate();
    }

    public double getEncoderRate() {
        return (getLeftEncoderRate() + getRightEncoderRate()) / 2;
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