package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import lib.components.LimitSwitch;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem m_instance;

    private WPI_TalonSRX m_flywheelA = new WPI_TalonSRX(LauncherConstants.kFlywheelMotorAPort);
    private WPI_TalonSRX m_flywheelB = new WPI_TalonSRX(LauncherConstants.kFlywheelMotorBPort);
    private SpeedControllerGroup m_flywheel = new SpeedControllerGroup(m_flywheelA, m_flywheelB);

    private Encoder m_encoder = new Encoder(LauncherConstants.kEncoderPorts[0], LauncherConstants.kEncoderPorts[1], LauncherConstants.kEncoderReversed);

    private LimitSwitch m_limitSwitch = new LimitSwitch(LauncherConstants.kLimitSwitchPort);

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(LauncherConstants.kS, LauncherConstants.kV, LauncherConstants.kA);
    private PIDController m_controller = new PIDController(LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD);

    double targetSpeedPer;
    double targetSpeedRPM;

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

    public void setSpeed(double speed) {
        setVoltage(
                m_feedforward.calculate(speed, Constants.IntakeConstants.kMaxAcc)
                        + m_controller.calculate(getEncoderRate(), speed));
    }

    public void stop() {
        set(0);
    }

    public void setTargetSpeedPer(double targetSpeedPer) {
        this.targetSpeedPer = targetSpeedPer;
    }

    public double getTargetSpeedPer() {
        return targetSpeedPer;
    }

    public void setTargetSpeedRPM(double targetSpeedPer) {
        this.targetSpeedPer = targetSpeedPer;
    }

    public double getTargetSpeedRPM() {
        return targetSpeedPer;
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

    public void resetController() {
        m_controller.reset();
    }
}
