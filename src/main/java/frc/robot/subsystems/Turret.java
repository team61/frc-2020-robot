package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private static Turret m_instance;

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(TurretConstants.kMotorPort);

    private TrapezoidProfile.Constraints m_constrants = new TrapezoidProfile.Constraints(TurretConstants.kMaxVel, TurretConstants.kMaxAcc);

    public static Turret getInstance() {
        if (m_instance == null) {
            m_instance = new Turret();
        }

        return m_instance;
    }

    public void set(double speed) {
        m_motor.set(speed);
    }

    public void stop() {
        set(0);
    }

    public TrapezoidProfile.Constraints getConstrants() {
        return m_constrants;
    }
}
