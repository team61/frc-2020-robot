package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private static Intake m_instance;

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(IntakeConstants.kMotorPort);

    private TrapezoidProfile.Constraints m_constrants = new TrapezoidProfile.Constraints(IntakeConstants.kMaxVel, IntakeConstants.kMaxAcc);

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake();
        }

        return m_instance;
    }

    public void set(double speed) {
        m_motor.set(speed);
    }

    public void stop() {
        set(0);
    }
}
