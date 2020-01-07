package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

public class Turret {

    private AHRS ahrs;
    public Turret() {
        try {
            ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error installing navX MXP: " + ex.getMessage(), true);
        }
        resetGryo();
    }

    /**
     *  Methods for Gyro Data
     * @return The displacement in degrees from -180 to 180
     * */
    public double getYaw() {
        return ahrs.getYaw();
    }

    public double getPitch() {
        return ahrs.getRoll();
    } // The gyro was inserted sideways into the robot

    public double getRoll() {
        return ahrs.getPitch();
    } // The gyro was inserted sideways into the robot

    public double getAccelerationX() {
        return ahrs.getRawAccelX();
    }

    public double getAccelerationY() {
        return ahrs.getRawAccelY();
    }

    public double getAccelerationZ() {
        return ahrs.getRawAccelZ();
    }

    public void resetGryo() {
        ahrs.reset();
    }

    public boolean isCalibrating() {
        return ahrs.isCalibrating();
    }
}
