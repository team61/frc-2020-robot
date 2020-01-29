package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

    private static TurretSubsystem m_instance;

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(TurretConstants.kMotorPort);

    private Encoder m_encoder = new Encoder(TurretConstants.kEncoderPorts[0], TurretConstants.kEncoderPorts[1], TurretConstants.kEncoderReversed);

    public static TurretSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new TurretSubsystem();
        }

        return m_instance;
    }

    public void set(double speed) {
        m_motor.set(speed);
    }

    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    public void stop() {
        m_motor.setVoltage(0);
    }

    public int getEncoder() {
        return m_encoder.get();
    }

    public double getEncoderRate() {
        return m_encoder.getRate();
    }

    public void resetEncoder() {
        m_encoder.reset();
    }
}
