package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;
import lib.components.LimitSwitch;

public class Feeder extends SubsystemBase {

    private static Feeder m_instance;

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(FeederConstants.kMotorPort);

    private LimitSwitch m_limitSwitch = new LimitSwitch(FeederConstants.kLimitSwitchPort);

    private Encoder m_encoder = new Encoder(FeederConstants.kEncoderPorts[0], FeederConstants.kEncoderPorts[1], FeederConstants.kEncoderReversed);

    public static Feeder getInstance() {
        if (m_instance == null) {
            m_instance = new Feeder();
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

    public boolean isSwitchSet() {
        return m_limitSwitch.isSwitchSet();
    }

    public int getEncoder() {
        return m_encoder.get();
    }

    public double getEncoderRate() {
        return m_encoder.getRate();
    }
}
