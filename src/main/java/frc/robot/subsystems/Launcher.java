package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import lib.components.LimitSwitch;

public class Launcher extends SubsystemBase {

    private static Launcher m_instance;

    private WPI_TalonSRX m_flywheelA = new WPI_TalonSRX(LauncherConstants.kFlywheelMotorAPort);
    private WPI_TalonSRX m_flywheelB = new WPI_TalonSRX(LauncherConstants.kFlywheelMotorBPort);
    private SpeedControllerGroup m_flywheel = new SpeedControllerGroup(m_flywheelA, m_flywheelB);

    private LimitSwitch m_limitSwitch = new LimitSwitch(LauncherConstants.kLimitSwitchPort);

    private double maxSpeed;

    public Launcher() {
        m_flywheelB.follow(m_flywheelA);
    }

    public static Launcher getInstance() {
        if (m_instance == null) {
            m_instance = new Launcher();
        }

        return m_instance;
    }

    public void set(double speed) {
        m_flywheel.set(speed);
    }

    public void stop() {
        set(0);
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public boolean isSwitchSet() {
        return m_limitSwitch.isSwitchSet();
    }
}
