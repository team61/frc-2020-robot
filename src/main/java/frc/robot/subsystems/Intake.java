package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private static Intake m_instance;

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(IntakeConstants.kMotorPort);

    private Encoder m_encoder = new Encoder(IntakeConstants.kEncoderPorts[0], IntakeConstants.kEncoderPorts[1], IntakeConstants.kEncoderReversed);


    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake();
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
        set(0);
    }

    public int getEncoder() {
        return m_encoder.get();
    }

    public double getEncoderRate() {
        return m_encoder.getRate();
    }
}
