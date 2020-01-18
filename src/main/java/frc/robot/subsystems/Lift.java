package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {

    private static Lift m_instance;

    private Compressor m_compressor = new Compressor(LiftConstants.kCompressorPort);

    private DoubleSolenoid m_solenoid = new DoubleSolenoid(LiftConstants.kSolenoidAPort, LiftConstants.kSolenoidBPort);


    public Lift() {
        m_compressor.setClosedLoopControl(true);
    }

    public static Lift getInstance() {
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

    public boolean isCompressorEnabled() {
        return m_compressor.enabled();
    }

    public boolean getPressureSwitchState() {
        return m_compressor.getPressureSwitchValue();
    }
}
