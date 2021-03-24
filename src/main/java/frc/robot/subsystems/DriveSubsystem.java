package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.function.DoubleConsumer;

public class DriveSubsystem extends SubsystemBase {

    private static DriveSubsystem m_instance;

    private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(DriveConstants.kFrontLeftPort);
    private final WPI_TalonFX m_leftSlave = new WPI_TalonFX(DriveConstants.kRearLeftPort);
    private final SpeedControllerGroup m_left = new SpeedControllerGroup(m_leftMaster, m_leftSlave);

    private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(DriveConstants.kFrontRightPort);
    private final WPI_TalonFX m_rightSlave = new WPI_TalonFX(DriveConstants.kRearRightPort);
    private final SpeedControllerGroup m_right = new SpeedControllerGroup(m_rightMaster, m_rightSlave);


    //public final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    //public final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);

    private AHRS m_ahrs; // NAVX

    private final DifferentialDriveOdometry m_odometry;

    private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_left, m_right);

    private double x = 0;
    private double y = 0;
    private boolean recording = false;
    private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    private NetworkTableEntry fileName =
    tab.add("File Name", "Output.txt").withWidget(BuiltInWidgets.kTextView)
       .getEntry();
       private NetworkTableEntry recordingState = tab.add("Recording", recording).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

  

public void setRecording(boolean value) {
    recording = value;
}

public boolean getRecording() {
    return recording;
}

    public String getFileName() {
        return fileName.getString("Output.txt");
    
    }

    public DriveSubsystem() {  
        //m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        //m_leftEncoder.reset();
        //m_rightEncoder.reset();
        
        try {
            
            File listFile = new File(AutoConstants.directory + "listFile.txt");
            Scanner reader = new Scanner(listFile);
    
            ArrayList<String> files = new ArrayList<String>();
            while (reader.hasNextLine()) {
              String data = reader.nextLine();
            files.add(data);
            }
            reader.close();
          } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
          }
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
       recordingState.setBoolean(recording);
       updateOdometry();
        x = getPose2d().getTranslation().getX();
        y = getPose2d().getTranslation().getY();
        //System.out.println("x: " + x + ", y: " + y);
        // System.out.println("angle: " + getPose2d());

        // SmartDashboard.putNumber("X", x);
        // SmartDashboard.putNumber("Y", y);
        // //SmartDashboard.putNumber("Velocity", getEncoderRate());
        // SmartDashboard.putNumber("Acceleration", getAccelerationX());
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

    public double getLeftMotorOutput() {
        return (m_leftMaster.getBusVoltage() + m_leftSlave.getBusVoltage()) / 2;
    }

    public double getRightMotorOutput() {
        return (m_rightMaster.getBusVoltage() + m_rightSlave.getBusVoltage()) / 2;
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_left.setVoltage(-leftVolts);
        m_right.setVoltage(rightVolts);
    }

  public void tankDriveVolt(double left, double right) {
      left = Math.copySign(left * left, left);
      right = Math.copySign(right * right, right);
        m_left.setVoltage(-left * 12);
        m_right.setVoltage(right * 12);
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
        return (m_leftMaster.getSelectedSensorPosition() + m_leftSlave.getSelectedSensorPosition()) / 2;
    }

    public double getRightEncoderDistance() {
        return -(m_rightMaster.getSelectedSensorPosition() + m_rightSlave.getSelectedSensorPosition()) / 2;
    }

    public double getLeftEncoderRate() {
        return (m_leftMaster.getSelectedSensorVelocity() + m_leftSlave.getSelectedSensorVelocity()) / 2;
    }

    public double getRightEncoderRate() {
        return -(m_rightMaster.getSelectedSensorVelocity() + m_rightSlave.getSelectedSensorVelocity()) / 2;
    }

    public double getEncoderRate() {
        return (getLeftEncoderRate() + getRightEncoderRate()) / 2;
    }

    public void resetLeftEncoder() {
        m_leftMaster.setSelectedSensorPosition(0);
        m_leftSlave.setSelectedSensorPosition(0);
    }

    public void resetRightEncoder() {
        m_rightMaster.setSelectedSensorPosition(0);
        m_leftSlave.setSelectedSensorPosition(0);
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