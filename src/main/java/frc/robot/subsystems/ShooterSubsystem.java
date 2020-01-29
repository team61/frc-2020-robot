package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem m_instance;

    private WPI_TalonSRX m_flywheelA = new WPI_TalonSRX(ShooterConstants.kFlywheelMotorAPort);
    private WPI_TalonSRX m_flywheelB = new WPI_TalonSRX(ShooterConstants.kFlywheelMotorBPort);
    private SpeedControllerGroup m_flywheel = new SpeedControllerGroup(m_flywheelA, m_flywheelB);

    public ShooterSubsystem() {
        m_flywheelB.follow(m_flywheelA);
    }

    public static ShooterSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new ShooterSubsystem();
        }

        return m_instance;
    }

    public void set(double speed) {
        m_flywheel.set(speed);
    }

    public void setVoltage(double voltage) {
        m_flywheel.setVoltage(voltage);
    }

    public void stop() {
        set(0);
    }
}
