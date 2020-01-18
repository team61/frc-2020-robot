package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private static Turret m_instance;

    private AHRS m_ahrs; // NAVX

    private WPI_TalonSRX m_heading = new WPI_TalonSRX(TurretConstants.kHeadingMotorPort);
    private WPI_TalonSRX m_angle = new WPI_TalonSRX(TurretConstants.kHeadingMotorPort);

    public Turret() {
        try {
            m_ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error installing navX MXP: " + ex.getMessage(), true);
        }
        resetGryo();
    }

    public static Turret getInstance() {
        if (m_instance == null) {
            m_instance = new Turret();
        }

        return m_instance;
    }

    public void setHeadingSpeed(double speed) {
        m_heading.set(speed);
    }

    public void setAngleSpeed(double speed) {
        m_angle.set(speed);
    }

    public void set(double headingSpeed, double angleSpeed) {
        setHeadingSpeed(headingSpeed);
        setAngleSpeed(angleSpeed);
    }

    public void stopHeading() {
        setHeadingSpeed(0);
    }

    public void stopAngle() {
        setHeadingSpeed(0);
    }

    public void stop() {
        stopHeading();
        stopAngle();
    }

    /**
     * Methods for Gyro Data
     *
     * @return The displacement in degrees from -180 to 180
     */

    public double getYaw() {
        return m_ahrs.getYaw();
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
}
