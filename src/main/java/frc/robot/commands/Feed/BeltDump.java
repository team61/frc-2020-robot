package frc.robot.commands.Feed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

import java.util.function.BooleanSupplier;

public class BeltDump extends CommandBase {

    private FeederSubsystem m_feederSubsystem;

    private BooleanSupplier[] m_solenoids;
    private double m_speed;

    public BeltDump(FeederSubsystem feederSubsystem, double speed, BooleanSupplier[] solenoids) {
        m_feederSubsystem = feederSubsystem;
        m_solenoids = solenoids;
        m_speed = speed;

        addRequirements(feederSubsystem);
    }

    @Override
    public void execute() {
        for (int i = 0; i < FeederConstants.kSolenoidPorts.length; i++) {
            if (m_solenoids[i].getAsBoolean()) {
                m_feederSubsystem.setSolenoidState(i, true);
            } else {
                m_feederSubsystem.setSolenoidState(i, false);
            }
        }
        m_feederSubsystem.set(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stop();
    }

}
