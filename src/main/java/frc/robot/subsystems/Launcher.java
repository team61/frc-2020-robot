package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import lib.components.LimitSwitch;

public class Launcher extends SubsystemBase {

    private static Launcher m_instance;

    private WPI_TalonSRX m_motorA = new WPI_TalonSRX(LauncherConstants.kMotorA);
    private WPI_TalonSRX m_motorB = new WPI_TalonSRX(LauncherConstants.kMotorB);
    private SpeedControllerGroup m_motorGroup = new SpeedControllerGroup(m_motorA, m_motorB);

    private LimitSwitch m_limitSwitch = new LimitSwitch(LauncherConstants.kLimitSwitchPort);

    private TrapezoidProfile.Constraints m_constrants = new TrapezoidProfile.Constraints(LauncherConstants.kMaxVel, LauncherConstants.kMaxAcc);

    private double speed;

    public Launcher() {
        m_motorB.follow(m_motorA);
    }

    public static Launcher getInstance() {
        if (m_instance == null) {
            m_instance = new Launcher();
        }

        return m_instance;
    }

    public void set(double speed) {
        m_motorGroup.set(speed);
    }

    public void stop() {
        set(0);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }

    public boolean isSwitchSet() {
        return m_limitSwitch.isSwitchSet();
    }
}
