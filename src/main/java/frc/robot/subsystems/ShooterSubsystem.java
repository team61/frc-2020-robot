package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem m_instance;

    private WPI_TalonSRX m_master = new WPI_TalonSRX(ShooterConstants.kMasterPort);
    private WPI_TalonSRX m_slave = new WPI_TalonSRX(ShooterConstants.kSlavePort);

    public ShooterSubsystem() {
        m_master.setInverted(true);
        m_slave.follow(m_master);
    }

    public static ShooterSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new ShooterSubsystem();
        }

        return m_instance;
    }

    public void set(double speed) {
        m_master.set(speed);
    }

    public void setVoltage(double voltage) {
        m_master.setVoltage(voltage);
    }

    public void stop() {
        set(0);
    }
}
