package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private AnalogGyro m_gyro = new AnalogGyro(TurretConstants.kGyroPort);

    protected SpeedController m_motor = new PWMTalonSRX(TurretConstants.kMotorPort);

    public Turret() {
        resetGryo();
    }

public void setSpeed(double speed) {
    m_motor.set(speed);
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
        return Rotation2d.fromDegrees(m_gyro.getAngle());
    }
    
    public void resetGryo() {
        m_gyro.reset();
    }
}
