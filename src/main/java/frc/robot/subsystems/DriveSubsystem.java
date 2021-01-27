package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final CANSparkMax m_leftMaster = new CANSparkMax(DriveConstants.kFrontLeftPort, MotorType.kBrushless);
    private final CANSparkMax m_leftSlave = new CANSparkMax(DriveConstants.kRearLeftPort,  MotorType.kBrushless);
    private final SpeedControllerGroup m_left = new SpeedControllerGroup(m_leftMaster, m_leftSlave);

    private final CANSparkMax m_rightMaster = new CANSparkMax(DriveConstants.kFrontRightPort, MotorType.kBrushless);
    private final CANSparkMax m_rightSlave = new CANSparkMax(DriveConstants.kRearRightPort, MotorType.kBrushless);
    private final SpeedControllerGroup m_right = new SpeedControllerGroup(m_rightMaster, m_rightSlave);


    //public final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    //public final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);

    private AHRS m_ahrs; // NAVX

    private final DifferentialDriveOdometry m_odometry;

    private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_left, m_right);

    private double x = 0;
    private double y = 0;

    public DriveSubsystem() {

        //m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        //m_leftEncoder.reset();
        //m_rightEncoder.reset();
        
        
        try {
            m_ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error installing navX MXP: " + ex.getMessage(), true);
        }
        resetGryo();

        m_odometry = new DifferentialDriveOdometry(getHeading(), AutoConstants.kStartingPosition);
        m_differentialDrive.setSafetyEnabled(false);
        m_differentialDrive.setDeadband(0); // Dead band is done with joysticks directly
     
    }

    @Override
    public void periodic() {
        // System.out.println("left: " + getLeftEncoderRate());
        // System.out.println("right: " + getRightEncoderRate());
        // System.out.println("total: " + getEncoderRate());
       
       updateOdometry();
        x = getPose2d().getTranslation().getX();
        y = getPose2d().getTranslation().getY();
        //System.out.println("x: " + x + ", y: " + y);
        System.out.println("angle: " + getPose2d());

        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        //SmartDashboard.putNumber("Velocity", getEncoderRate());
        SmartDashboard.putNumber("Acceleration", getAccelerationX());
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
       m_differentialDrive.tankDrive(-leftSpeed, -rightSpeed, squaredInputs);
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
        m_left.setVoltage(leftVolts);
        m_right.setVoltage(-rightVolts);
    }


    public void setMaxOutput(double maxOutput) {
        m_differentialDrive.setMaxOutput(maxOutput);
    }

    /**
     * Methods for Encoder Data
     **/

    // public void setDistancePerPulse() {
    //     m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    //     m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // }

    public double getLeftEncoderDistance() {
        return (m_leftMaster.getEncoder().getPosition() + m_leftSlave.getEncoder().getPosition()) / 2 / 28.07;
    }

    public double getRightEncoderDistance() {
        return -(m_rightMaster.getEncoder().getPosition() + m_rightSlave.getEncoder().getPosition()) / 2 / 28.07;
    }

    public double getLeftEncoderRate() {
        return (m_leftMaster.getEncoder().getVelocity() + m_leftSlave.getEncoder().getVelocity()) / 2 / 1750;
    }

    public double getRightEncoderRate() {
        return -(m_rightMaster.getEncoder().getVelocity() + m_rightSlave.getEncoder().getVelocity()) / 2 / 1750;
    }

    public double getEncoderRate() {
        return (getLeftEncoderRate() + getRightEncoderRate()) / 2;
    }

    public void resetLeftEncoder() {
        m_leftMaster.getEncoder().setPosition(0);
        m_leftSlave.getEncoder().setPosition(0);
    }

    public void resetRightEncoder() {
        m_rightMaster.getEncoder().setPosition(0);
        m_rightSlave.getEncoder().setPosition(0);
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
        return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
    }

    public void updateOdometry() {
        m_odometry.update(getHeading(), getLeftEncoderDistance(), getRightEncoderDistance());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_ahrs.reset();
        m_odometry.resetPosition(pose, getHeading());
    }
}