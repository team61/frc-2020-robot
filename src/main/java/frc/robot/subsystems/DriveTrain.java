package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    private static final double WHEEL_DIAMETER = 8;
    private static final double PULSE_PER_REVOLUTION = 1440;
    private static final double ENCODER_GEAR_RATIO = 1;
    private static final double GEAR_RATIO = 12.75;

    private static final double distancePerPulse = Math.PI * WHEEL_DIAMETER / PULSE_PER_REVOLUTION / ENCODER_GEAR_RATIO / GEAR_RATIO;

    private DifferentialDrive m_differentialDrive;
    protected SpeedController mFrontLeft;
    protected SpeedController mRearLeft;
    protected SpeedControllerGroup mLeftStack;

    protected SpeedController mFrontRight;
    protected SpeedController mRearRight;
    protected SpeedControllerGroup mRightStack;

    protected Encoder leftEncoder;
    protected Encoder rightEncoder;

    private Solenoid sPTOA;
    private Solenoid sPTOB;

    private boolean PTOState;

    public DriveTrain() {

        mFrontLeft = new PWMTalonSRX(Constants.mFrontLeft);
        mRearLeft = new PWMTalonSRX(Constants.mRearLeft);
        mLeftStack = new SpeedControllerGroup(mFrontLeft, mRearLeft);

        mFrontRight = new PWMTalonSRX(Constants.mFrontRight);
        mRearRight = new PWMTalonSRX(Constants.mRearRight);
        mLeftStack = new SpeedControllerGroup(mFrontRight, mRearRight);

        sPTOA = new Solenoid(Constants.pcmModule, Constants.sPTOA);
        sPTOB = new Solenoid(Constants.pcmModule, Constants.sPTOB);

        leftEncoder = new Encoder(Constants.eLeftA, Constants.eLeftB, false);
        rightEncoder = new Encoder(Constants.eRightA, Constants.eRightB, true);

        m_differentialDrive = new DifferentialDrive(mLeftStack, mRightStack);
        m_differentialDrive.setSafetyEnabled(false);

        PTOState = false;
    }

    /**
     * Power Transfer Option (PTO) transfers motor control from the drive train to the climber and vise versa.
     * */

    public void setPTOState(boolean PTOState) {
        sPTOA.set(PTOState);
        sPTOB.set(!PTOState);
        this.PTOState = PTOState;
    }

    public boolean getPTOState() {
        return PTOState;
    }

    /**
     * Movement commands
     * */

    public void moveLeft(double speed) {
        mLeftStack.set(speed);
    }

    public void moveRight(double speed) {
        mRightStack.set(speed);
    }

    public void stopLeft() {
        mLeftStack.stopMotor();
    }

    public void stopRight() {
        mRightStack.stopMotor();
    }

    /**
     * Methods for Encoder Data
     **/

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
    

    public void setDistancePerPulse() {
        leftEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setDistancePerPulse(distancePerPulse);
    }

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
}
