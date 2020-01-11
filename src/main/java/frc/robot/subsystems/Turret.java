package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private AnalogGyro m_gyro = new AnalogGyro(TurretConstants.kGyroPort);

    private SpeedController m_motor = new PWMTalonSRX(TurretConstants.kMotorPort);

    private DriveTrain m_driveTrain;
    private Pose2d m_position;


    public Turret(DriveTrain driveTrain) {
        resetGryo();
        m_driveTrain = driveTrain;
        updatePose2d();
        
    }

public void updatePose2d() {
    m_position = new Pose2d(new Translation2d(m_driveTrain.getX() + TurretConstants.kXDistanceFromRobot, 
    m_driveTrain.getY() + TurretConstants.kYDistanceFromRobot), getHeading()); 
}

public Pose2d getPose2d() {
    return m_position;
}

public Translation2d getTranslation2d() {
    return m_position.getTranslation();
}

public double getX() {
    return getTranslation2d().getX();
}

public double getY() {
    return getTranslation2d().getY();
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
