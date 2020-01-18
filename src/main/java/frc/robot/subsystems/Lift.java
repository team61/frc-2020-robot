package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {

    private static Lift m_instance;

    DoubleSolenoid liftSolenoid;

    public Lift() {
        liftSolenoid = new DoubleSolenoid(LiftConstants.kSolenoidAPort, LiftConstants.kSolenoidBPort);
    }

    public static Lift getInstance() {
        return m_instance;
    }

    /** Extends the lift solenoid. */
    public void extend() {
        liftSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    /** Retracts the lift solenoid. */
    public void retract() {
        liftSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
