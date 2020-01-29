package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {

    private static LiftSubsystem m_instance;

    private DoubleSolenoid m_solenoid = new DoubleSolenoid(LiftConstants.kSolenoidAPort, LiftConstants.kSolenoidBPort);

    public static LiftSubsystem getInstance() {
        return m_instance;
    }

    /** Extends the lift solenoid. */
    public void extend() {
        m_solenoid.set(DoubleSolenoid.Value.kForward);
    }

    /** Retracts the lift solenoid. */
    public void retract() {
        m_solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void off() {
        m_solenoid.set(DoubleSolenoid.Value.kOff);
    }
}
