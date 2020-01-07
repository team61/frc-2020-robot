package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

    private static final double WHEEL_DIAMETER = 8;
    private static final double PULSE_PER_REVOLUTION = 1440;
    private static final double ENCODER_GEAR_RATIO = 1;
    private static final double GEAR_RATIO = 12.75;

    private static final double distancePerPulse = Math.PI * WHEEL_DIAMETER / PULSE_PER_REVOLUTION / ENCODER_GEAR_RATIO / GEAR_RATIO;

    protected SpeedController mFrontLeft;
    protected SpeedController mRearLeft;
    protected SpeedControllerGroup mLeftStack;

    protected SpeedController mFrontRight;
    protected SpeedController mRearRight;
    protected SpeedControllerGroup mRightStack;

    protected Encoder leftEncoder;
    protected Encoder rightEncoder;

    private DifferentialDrive m_differentialDrive;

    public DriveTrain() {

        mFrontLeft = new PWMTalonSRX(DriveConstants.mFrontLeft);
        mRearLeft = new PWMTalonSRX(DriveConstants.mRearLeft);
        mLeftStack = new SpeedControllerGroup(mFrontLeft, mRearLeft);

        mFrontRight = new PWMTalonSRX(DriveConstants.mFrontRight);
        mRearRight = new PWMTalonSRX(DriveConstants.mRearRight);
        mRightStack = new SpeedControllerGroup(mFrontRight, mRearRight);

        leftEncoder = new Encoder(DriveConstants.eLeftA, DriveConstants.eLeftB, false);
        rightEncoder = new Encoder(DriveConstants.eRightA, DriveConstants.eRightB, true);

        m_differentialDrive = new DifferentialDrive(mLeftStack, mRightStack);
        m_differentialDrive.setSafetyEnabled(false);
    }


    /**
     * Movement commands
     * */

    public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
        m_differentialDrive.tankDrive(leftSpeed, rightSpeed, squaredInputs);
    }
    public void tankDrive(double leftSpeed, double rightSpeed) {
        tankDrive(leftSpeed, rightSpeed, true);
    }

    public void tankDrive(double speed) {
        tankDrive(speed, speed);
    }

    public void stopTankDrive() {
        tankDrive(0);
    }

    /**
     * Methods for Encoder Data
     **/

    public void setDistancePerPulse() {
        leftEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setDistancePerPulse(distancePerPulse);
    }

    public double getLeftEncoder() {
        return leftEncoder.getDistance();
    }

    public double getFrontEncoder() {
        return getLeftEncoder();
    }

    public double getRightEncoder() {
        return rightEncoder.getDistance();
    }

    public double getRearEncoder() {
        return getRightEncoder();
    }

    public void resetLeftEncoder() {
        leftEncoder.reset();
    }

    public void resetRightEncoder() {
        rightEncoder.reset();
    }

    public void resetEncoders() {
        resetRightEncoder();
        resetLeftEncoder();
    }

    public void resetRearEncoder() {
        resetRightEncoder();
    }

    public void resetFrontEncoder() {
        resetLeftEncoder();
    }

    public double getDistanceTraveled() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }
}
