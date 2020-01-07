package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import javax.management.ListenerNotFoundException;

public class Lift extends SubsystemBase {

    DoubleSolenoid liftSolenoid;

    public Lift() {
        liftSolenoid = new DoubleSolenoid(Constants.LiftConstants.liftSolenoidA, Constants.LiftConstants.liftSolenoidB);
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
