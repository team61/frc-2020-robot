package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem m_instance;

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(IntakeConstants.kMotorPort);

    private Encoder m_encoder = new Encoder(IntakeConstants.kEncoderPorts[0], IntakeConstants.kEncoderPorts[1], IntakeConstants.kEncoderReversed);

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);
    private PIDController m_controller = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

    public static IntakeSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new IntakeSubsystem();
        }

        return m_instance;
    }

    public void set(double speed) {
        m_motor.set(speed);
    }

    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    public void setSpeed(double speed) {
        setVoltage(
                m_feedforward.calculate(speed, IntakeConstants.kMaxAcc)
                        + m_controller.calculate(getEncoderRate(), speed));
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

    public void resetEncoder() {
        m_encoder.reset();
    }

    public void resetController() {
        m_controller.reset();
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }
}
